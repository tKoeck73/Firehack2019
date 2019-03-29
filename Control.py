from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.Circle import Circle
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.Waypoint import Waypoint
from afrl.cmasi.VehicleActionCommand import VehicleActionCommand
from afrl.cmasi.LoiterAction import LoiterAction
from afrl.cmasi.LoiterType import LoiterType
from afrl.cmasi.LoiterDirection import LoiterDirection
from afrl.cmasi.CommandStatusType import CommandStatusType
from afrl.cmasi.AltitudeType import AltitudeType
from afrl.cmasi.searchai.HazardZoneDetection import HazardZoneDetection
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.Location3D import Location3D

from afrl.cmasi.AirVehicleState import AirVehicleState
from afrl.cmasi.FlightDirectorAction import FlightDirectorAction
import time

def sgn(x):
	return (x-abs(x))/abs(x) + 1

class Drone():
	"""Stores data about the drones"""
	def __init__(self,id):
		self.__id = id
		self.__count = 0
		self.__comparison = 15
		self.lastHeading = 0 #Do not rely on these to be accurate, use received data wherever possible
		self.heading = 0
		self.successiveData = 0 #Used to detect whether the drone is in the fire region
		self.trueHeading = 0

	def get_ID(self):
		return self.__id

	def get_Count(self):
		return self.__count

	def get_Comparison(self):
		return self.__comparison

	def update_Count(self,amount,modulo):
		self.__count = (self.__count + amount) % modulo

	def update_Comparison(self,val):
		self.__comparison = val


class PrintLMCPObject(IDataReceived):
	def dataReceived(self, lmcpObject):
		print(lmcpObject.toXMLStr(""))


class Controller(IDataReceived):

	def __init__(self, tcpClient):
		self.__client = tcpClient
		self.__estimatedHazardZone = Polygon()
		self.vehicles = {} #dictionary with id for key and Drone object as value


	def catchNewDrone(self,id):
		"""Add a new drone object to the controller's list"""
		assert(id not in self.vehicles)
		self.vehicles[id] = Drone(id)
		print("caught drone " +  str(id))

	def updateHeading(self,id,heading):

		self.vehicles[id].lastHeading = self.vehicles[id].heading
		self.vehicles[id].heading = heading

		actCom = VehicleActionCommand()
		actCom.set_VehicleID(id)
		actCom.set_Status(CommandStatusType.Pending)
		actCom.set_CommandID(1)

		directorAction = FlightDirectorAction()
		directorAction.set_Heading(heading)
		directorAction.set_Speed(20)

		actCom.get_VehicleActionList().append(directorAction)
		self.__client.sendLMCPObject(actCom)
		print("sent UAV" + str(id) + " on heading " + str(heading) + ".")

	def dataReceived(self, lmcpObject):

		#handle air vehicle status data
		if isinstance(lmcpObject,AirVehicleState):
			state = lmcpObject
			id = state.get_ID()

			if id not in self.vehicles:
				self.catchNewDrone(id)

			self.vehicles[id].trueHeading = state.get_Heading()
			self.vehicles[id].successiveData+=1
			#counts that another status update for drone [id] occured
			self.vehicles[id].update_Count(1,self.vehicles[id].get_Comparison())
			#print(self.vehicles[id].get_Count())

			if self.vehicles[id].get_Count() == 0:
				#print("updatig heading")
				self.updateHeading(id,(state.get_Heading()+90)%360)
				self.vehicles[id].update_Comparison(self.vehicles[id].get_Comparison()+2)

		#handle hazard detecction data
		if isinstance(lmcpObject, HazardZoneDetection):
			hazardDetected = lmcpObject
			#Get location where zone first detected
			detectedLocation = hazardDetected.get_DetectedLocation()
			#Get entity that detected the zone
			detectingEntity = hazardDetected.get_DetectingEnitiyID()
			if detectingEntity == 0:
				return
			self.vehicles[detectingEntity].update_Comparison(15)
			if self.vehicles[detectingEntity].successiveData > 5 :
				print("deflecting UAV" + str(detectingEntity) + " data count: " + str(self.vehicles[detectingEntity].successiveData))
				head = self.vehicles[detectingEntity].trueHeading
				lhead = self.vehicles[detectingEntity].lastHeading
				print("last heading: "+ str(lhead)+ " current heading: " + str(head))
				newHead = head + 90*(sgn(lhead - head))
				newHead = newHead % 360
				self.updateHeading(detectingEntity,newHead)
				self.__estimatedHazardZone.get_BoundaryPoints().append(detectedLocation)
			self.vehicles[detectingEntity].successiveData = 0
			#Note: Polygon points must be in clockwise or counter-clockwise order to get a shape without intersections

			#Send out the estimation report to draw the polygon
			self.sendEstimateReport();

			#print('UAV' + str(detectingEntity) + ' detected hazard at ' + str(detectedLocation.get_Latitude()) + ',' + str(detectedLocation.get_Longitude()));

	def sendEstimateReport(self):
		#Setting up the mission to send to the UAV
		hazardZoneEstimateReport = HazardZoneEstimateReport()
		hazardZoneEstimateReport.set_EstimatedZoneShape(self.__estimatedHazardZone)
		hazardZoneEstimateReport.set_UniqueTrackingID(1)
		hazardZoneEstimateReport.set_EstimatedGrowthRate(0)
		hazardZoneEstimateReport.set_PerceivedZoneType(HazardType.Fire)
		hazardZoneEstimateReport.set_EstimatedZoneDirection(0)
		hazardZoneEstimateReport.set_EstimatedZoneSpeed(0)

		#Sending the Vehicle Action Command message to AMASE to be interpreted
		self.__client.sendLMCPObject(hazardZoneEstimateReport)


#################
## Main
#################

if __name__ == '__main__':
	myHost = 'localhost'
	myPort = 5555
	amaseClient = AmaseTCPClient(myHost, myPort)
	#amaseClient.addReceiveCallback(PrintLMCPObject())
	amaseClient.addReceiveCallback(Controller(amaseClient))

	try:
		# make a threaded client, listen until a keyboard interrupt (ctrl-c)
		#start client thread
		amaseClient.start()

		while True:
			pass
	except KeyboardInterrupt as ki:
		print("Stopping amase tcp client")
	except Exception as ex:
		print(ex)
	amaseClient.stop()
