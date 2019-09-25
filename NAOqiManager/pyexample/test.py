from naoqipy import *

image = QIImage(width=160, height=45, layers=3,
                color_space=0, data=[2, 3, 3, 3])

print(QIImage.calcBufferSize(160, 45, 3))
print(image.width)
print(image.height)
print(image.layers)
print(image.color_space)
print(image.camera_id)
print(image.buffer_size)
print(image.time_stamp)
print(image.data[:10])

image.dumpFile("b.data")

image = QIImage.fromFile("a.data")
print(image.width)
print(image.height)
print(image.layers)
print(image.color_space)
print(image.camera_id)
print(image.buffer_size)
print(image.time_stamp)
print(image.data[:10])

stereoCamera = QICameraClient("StereoCamera")
stereoCamera.openSharedMemory()
image = stereoCamera.getImage()
print(stereoCamera.name)
print(image.width)
print(image.height)
print(image.layers)
print(image.color_space)
print(image.camera_id)
print(image.buffer_size)
print(image.time_stamp)
print(image.data[:10])
print(image.data_numpy.shape)
print(image.data_numpy)

data = QIGazeAnalysisData(0.666, 23333)
print(data.looking_at_score, data.time_stamp, data.isLookingAt())

gazeAnalysiser = QIGazeAnalysisClient("gazeAnalysiser")
gazeAnalysiser.openSharedMemory()
data = gazeAnalysiser.getData()
print(data.looking_at_score, data.time_stamp, data.isLookingAt())
