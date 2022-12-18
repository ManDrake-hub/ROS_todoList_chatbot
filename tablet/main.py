from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from apscheduler.schedulers.background import BackgroundScheduler
import random

# Creating a flask app and using it to instantiate a socket object
app = Flask(__name__)
socketio = SocketIO(app)

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
  	<h1>TO DO LIST</h1>
  </head>

<body style="background-color:rgb(40, 42, 54);">

  
  <table>
  	<!--Entità tabella-->
    <tr>
    	<!--Header delle colonne-->
      <th>Nome</th>
      <th>Scadenza</th>
    </tr>
    <tr>
    	<!--Contenuto prima riga-->
      <td>Alfreds Futterkiste</td>
      <td>Maria Anders</td>
    </tr>
    <tr>
   	<!--Contenuto seconda riga-->
      <td>Centro comercial Moctezuma</td>
      <td>Francisco Chang</td>
    </tr>
  </table>
  </p>""" + str(result) +"""
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
    
