from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from apscheduler.schedulers.background import BackgroundScheduler
import random

# Creating a flask app and using it to instantiate a socket object
app = Flask(__name__)
socketio = SocketIO(app)

todo_list = []
def execute():
    x = 'this is imam'
    #perform some task after first print then print next function
    y = 'this akarsh'
    #perform some task after first print then print next function
    z = str(random.random())
    todo_list.append(x+y+z)
    result = todo_list
    return result


@app.route('/')
def home():
    with app.app_context():
        return render_template('todo_list.html', x=execute())

#schedule job
scheduler = BackgroundScheduler()
running_job = scheduler.add_job(home, 'interval', seconds=4, max_instances=1)

scheduler.start()

# Notice how socketio.run takes care of app instantiation as well.
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0')
    print("ok")
    