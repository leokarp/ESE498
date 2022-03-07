from picamera import PiCamera
camera = PiCamera()
camera.capture('testing1.png')
camera.resolution = (320,208)
camera.capture('testing2.png')