from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from apscheduler.schedulers.background import BackgroundScheduler
import random

# Creating a flask app and using it to instantiate a socket object
app = Flask(__name__)
socketio = SocketIO(app)

style = """<style>
    body {
    padding-left: 11em;
    font-family: Georgia, "Times New Roman",
          Times, serif;
    color: purple;
    background-color: #d8da3d }
    ul.navbar {
      list-style-type: none;
      padding: 0;
      margin: 0;
      position: absolute;
      top: 2em;
      left: 1em;
      width: 9em }
    h1 {
      font-family: Helvetica, Geneva, Arial,
            SunSans-Regular, sans-serif }
    ul.navbar li {
      background: white;
      margin: 0.5em 0;
      padding: 0.3em;
      border-right: 1em solid black }
    ul.navbar a {
      text-decoration: none }
    a:link {
      color: blue }
    a:visited {
      color: purple }
    address {
      margin-top: 1em;
      padding-top: 1em;
      border-top: thin dotted }
  </style>"""

def execute():
    x = """this is imam<br/>
    """
    #perform some task after first print then print next function
    y = """this akarsh
    """
    #perform some task after first print then print next function
    z = str(random.random())
    result = [x,y,z]
    return style+"""
    <head>
	<title></title>
</head>
<body>
    <h3>This is a parameter from app</h3>
    """ + str(result) +"""
</body>"""


@app.route('/')
def home():
    with app.app_context():
        return render_template('index.html', x=execute())

#schedule job
scheduler = BackgroundScheduler()
running_job = scheduler.add_job(home, 'interval', seconds=4, max_instances=1)
scheduler.start()

# Notice how socketio.run takes care of app instantiation as well.
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0')
    print("ok")
    