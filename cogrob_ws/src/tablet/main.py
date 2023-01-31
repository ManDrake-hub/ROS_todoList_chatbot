from flask import Flask, render_template
from flask_socketio import SocketIO
from apscheduler.schedulers.background import BackgroundScheduler
from optparse import OptionParser
from utils import get_page


flag = False
app = Flask(__name__)
socketio = SocketIO(app)


@app.route('/')
def home():
    with app.app_context():
        return render_template('index.html', x=get_page())


# Schedule the update of the webpage
scheduler = BackgroundScheduler()
running_job = scheduler.add_job(home, 'interval', seconds=2, max_instances=1)
scheduler.start()


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    socketio.run(app, host='0.0.0.0')