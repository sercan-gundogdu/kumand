import subprocess
import os
import re
import socket

def getDeviceAddr():
	ip_addr_format = "^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$"
	phy_addr_format = "^[0-9a-f]{2}\-[0-9a-f]{2}\-[0-9a-f]{2}\-[0-9a-f]{2}\-[0-9a-f]{2}\-[0-9a-f]{2}$"

	net = socket.gethostbyname(socket.gethostname()).split('.')
	for counter in range(256):
		subprocess.Popen(["ping", "-n", "1", "%s.%s.%s."%(net[0], net[1], net[2]) + str(counter)], \
				creationflags=0x08000000, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			
	arp_table = subprocess.Popen(["arp", "-a"], \
		creationflags=0x08000000, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	line = str(arp_table.communicate())

	file = open("./globals/device", "r")
	phy_addr = file.read()
	file.close()

	phy_addr_index = line.find(phy_addr)
	ip_addr_index = phy_addr_index - 22
	ip_addr = line[ip_addr_index : ip_addr_index + 15].strip(" ")
	try:
		return re.match(ip_addr_format, ip_addr).group(0)
	except AttributeError:
		return False

if __name__ == "__main__": 
	print(getDeviceAddr())