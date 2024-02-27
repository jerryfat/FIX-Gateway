import requests
import argparse


# Parse connection argument
parser = argparse.ArgumentParser()
#parser.add_argument("-c", "--connect", help="connection string")
parser.add_argument("-m", "--module", help="module name to find deps for")
args = parser.parse_args() # command line parameters

if args.module: # input from sitl qgcs or usb serial port to autopilot
    #connection_string = args.px4
    module = args.module
    print("module = ", module )

your_package = args.module
pypi_url = 'https://pypi.python.org/pypi/' + your_package + '/json'
data = requests.get(pypi_url).json()

reqs = data['info']['requires_dist']
print("reqs   = ",reqs)

