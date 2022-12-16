from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from flask import Response, url_for, current_app
from apscheduler.schedulers.background import BackgroundScheduler
import random
import threading
from pyimagesearch.motion_detection.singlemotiondetector import SingleMotionDetector
from imutils.video import VideoStream
import cv2
import face_recognition
import numpy as np
import imutils

# Creating a flask app and using it to instantiate a socket object
app = Flask(__name__)
app.config['SERVER_NAME'] = '127.0.0.1:5000'

socketio = SocketIO(app)
lock = threading.Lock()
vs = VideoStream(src=0).start()
obama_image = face_recognition.load_image_file("obama.jpg")
obama_face_encoding = face_recognition.face_encodings(obama_image)[0]

# Load a second sample picture and learn how to recognize it.
biden_image = face_recognition.load_image_file("biden.jpg")
biden_face_encoding = face_recognition.face_encodings(biden_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [
    obama_face_encoding,
    biden_face_encoding
]
known_face_names = [
    "Barack Obama",
    "Joe Biden"
]

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []

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

def detect_motion(frameCount):
	# grab global references to the video stream, output frame, and
	# lock variables
	global vs, outputFrame, lock
	# initialize the motion detector and the total number of frames
	# read thus far
	md = SingleMotionDetector(accumWeight=0.1)
	total = 0

	# loop over frames from the video stream
	while True:
		process_this_frame = True
		# read the next frame from the video stream, resize it,
		# convert the frame to grayscale, and blur it
		frame = vs.read()
		frame = imutils.resize(frame, width=600)
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (7, 7), 0)
		if process_this_frame:
        	# Resize frame of video to 1/4 size for faster face recognition processing
			small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
			rgb_small_frame = small_frame[:, :, ::-1]
			face_locations = face_recognition.face_locations(rgb_small_frame)
			face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
			face_names = []
			for face_encoding in face_encodings:
            	# See if the face is a match for the known face(s)
				matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
				name = "Unknown"
				face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
				best_match_index = np.argmin(face_distances)
				if matches[best_match_index]:
					name = known_face_names[best_match_index]
				face_names.append(name)
		process_this_frame = not process_this_frame
		# Display the result

		# if the total number of frames has reached a sufficient
		# number to construct a reasonable background model, then
		# continue to process the frame
		if total > frameCount:
			# detect motion in the image
			motion = md.detect(gray)
			# check to see if motion was found in the frame
			#if motion is not None:
				# unpack the tuple and draw the box surrounding the
				# "motion area" on the output frame
				#(thresh, (minX, minY, maxX, maxY)) = motion
				#cv2.rectangle(frame, (minX, minY), (maxX, maxY),
				#	(0, 0, 255), 2)
			for (top, right, bottom, left), name in zip(face_locations, face_names):
				top *= 4
				right *= 4
				bottom *= 4
				left *= 4
	
        		# Draw a box around the face
				cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
	
        		# Draw a label with a name below the face
				cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
				font = cv2.FONT_HERSHEY_DUPLEX
				cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
		
		# update the background model and increment the total number
		# of frames read thus far
		md.update(gray)
		total += 1
		# acquire the lock, set the output frame, and release the
		#lock
		with lock:
			outputFrame = frame.copy()
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
def generate():
    # grab global references to the output frame and lock variables
	global outputFrame, lock
	# loop over frames from the output stream
	while True:
		# wait until the lock is acquired
		with lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			if outputFrame is None:
				continue
			# encode the frame in JPEG format
			(flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
			# ensure the frame was successfully encoded
			if not flag:
				continue
		# yield the output frame in the byte format
		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
			bytearray(encodedImage) + b'\r\n')

@app.route("/video_feed")
def video_feed():
	# return the response generated along with the specific media
	# type (mime type)
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

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
    t = threading.Thread(target=detect_motion, args=(
	  	32,))
    t.daemon = True
    t.start()
    socketio.run(app, host='0.0.0.0')
    print("ok")
vs.stop()
    