
from flask import Flask, render_template
from flask_socketio import SocketIO
from apscheduler.schedulers.background import BackgroundScheduler
import os
from actions.ToDo import CustomUnpickler, ToDo
from std_msgs.msg import Int16MultiArray, Bool
import datetime
import rospy
from std_msgs.msg import String
from optparse import OptionParser
from utils import Session

flag = False
app = Flask(__name__)
socketio = SocketIO(app)
    

def callback(value):
    global actual_user
    actual_user = value.data

def read_user()-> str:
    with open("../../../../../.ros/name.txt","r") as f:
        return f.read().split("\n")[0]

def get_todo_data():
    try:
        user=read_user()
        print(f"utente{user}")
    except Exception as e:
        print(e)
        user = "default"
    todo_path = f"../../../chatbot/todo_{user}.pickle"
    try:
        todo: ToDo = CustomUnpickler(open(todo_path, "rb")).load()
    except Exception as e:
        todo_path = f"../../../chatbot/todo_default.pickle"
        todo: ToDo = CustomUnpickler(open(todo_path, "rb")).load()
    tab_data = []
    if todo._todo:
        for c in todo.get_categories():
            tasks = todo.get_tasks_of_category(c)
            for t in tasks:
                tab_data.append((c, t.tag, t.deadline if t.deadline is not None else "", 
                                    t.alarm))
    return tab_data

def get_todo_data_path(path):
    tab_data = []
    todo_path = path
    todo: ToDo = CustomUnpickler(open(todo_path, "rb")).load()
    if todo._todo:
        for c in todo.get_categories():
            tasks = todo.get_tasks_of_category(c)
            for t in tasks:
                tab_data.append((c, t.tag, t.deadline if t.deadline is not None else "", 
                                    t.alarm))
    return tab_data


def get_rows_from_data(data):
    rows = []
    for d in data:
        rows.append("<tr>" + " ".join(["<td>" + str(x) + "</td>" for x in d[:-1]]) + "</tr>")
    return "\n".join(rows)

def check_alerts(todo_dir, alert_length: int=15):
    data_with_alerts = []
    todo_paths = [x for x in os.listdir(todo_dir) if x.endswith(".pickle")]
    for todo_path in todo_paths:
        data = get_todo_data_path(todo_dir + todo_path)
        for d in data:
            if d[-1] is not None and datetime.datetime.now() >= d[-1] and datetime.datetime.now() <= d[-1] + datetime.timedelta(seconds=alert_length):
                data_with_alerts.append(d)
    return data_with_alerts

style = """
        <style>
        table, th, td {
            border: 1px solid;
            color : #8be9fd;
            margin: auto;
            width: 50%;
            padding: 10px;
            border-radius: 25px;
        }
        th{
            color : #8be9fd;
            font-weight: bold;
        }
        td{
            color: #f8f8f2;
            border-color: #ff79c6;
        }
        h1{
            margin: auto;
            width: 50%;
            padding: 10px;
            color: #50fa7b;
            font-weight: bold;
            text-align: center;
        }
        </style>"""

body = """
        <head><h1>TO DO LIST</h1></head>
        <body style="background-color:rgb(40, 42, 54);">
        """

body_closure = """</body>"""

def execute():
    global actual_user
    data = get_todo_data()
    rows = get_rows_from_data(data)
    rows_alert = get_rows_from_data(check_alerts("../../../chatbot/"))

    table_tasks = f"""
            <table>
            <tr>
                <!--Header delle colonne-->
                <th>Categoria</th>
                <th>Nome</th>
                <th>Scadenza</th>
            </tr>
            {rows}
            </table>
            </p>
            """
    table_alarms = f"""
            <h1>Alerts</h1> <table>
            <tr>
    	        <!--Header delle colonne-->
                <th>Categoria</th>
                <th>Nome</th>
                <th>Scadenza</th>
            </tr>
            {rows_alert}
            </table>
            </p>
            """ if rows_alert else ""
    return style + body + table_tasks + table_alarms + body_closure

@app.route('/')
def home():
    with app.app_context():
        return render_template('index.html', x=execute())

scheduler = BackgroundScheduler()
running_job = scheduler.add_job(home, 'interval', seconds=4, max_instances=1)
scheduler.start()

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    socketio.run(app, host='0.0.0.0')
