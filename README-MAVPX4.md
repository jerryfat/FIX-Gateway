
# Mavlink to PyG5 and pyEfis using my MAV to pickle dict converter

![](/home/jf/testgit/FIX-Gateway/Screenshot from 2023-11-30 17-57-49.png)


=================================================================================================
##Installing Dronekit
roneKit-Python can be installed on a Linux, Mac OSX, or Windows computer that has Python 2.7 and can install Python packages from the Internet.

It is installed from pip on all platforms:

pip install dronekit

Installation notes:

    Mac and Linux require you prefix the command with sudo.

    sudo pip install dronekit

    On Linux you may need to first install pip and python-dev:

    sudo apt-get install python-pip python-dev

    Alternatively, the easy_install tool provides another way to install pip:

    sudo easy_install pip


add to  File "/home/jf/.local/lib/python3.10/site-packages/dronekit/__init__.py", line 49, in <module>
replace line "from collections     import MutableMapping"
=================================================================================================

"
# Import DroneKit-Python
from platform import python_version
print(python_version())

try: # for dronekit after python 3.8
    # 👇️ using Python 3.10+
    from collections.abc import MutableMapping
except ImportError:
    # 👇️ using Python 3.10-
    from collections     import MutableMapping

# 👇️ <class 'collections.abc.MutableMapping'>
print(MutableMapping)
from dronekit import connect, Command, LocationGlobal 
"
to test:
$ python
Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import dronekit
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/home/jf/.local/lib/python3.10/site-packages/dronekit/__init__.py", line 49, in <module>
    from collections     import MutableMapping
ImportError: cannot import name 'MutableMapping' from 'collections' (/usr/lib/python3.10/collections/__init__.py)
>>>
>>> from collections.abc import MutableMapping (no error if python 3.10+)
>>> from collections     import MutableMapping (no error if python 3.10-)


=================================================================================================
##Installation of pyG5
https://github.com/blauret/pyG5/blob/main/README.md
pyG5 depends on pySide6. Due to failure to install pySide6 from pip on Raspberry Pi OS it is not a dependency of the pyG5. As a result it needs to be installed manually.

        > sudo pip3 install pyside6

The install PyG5:

        > sudo pip3 install pyG5

Running

        > pyG5DualStacked

Running on Raspberry Pi it is recommended to install FreeSans fonts in order to be consistent with the rendering on the current main development platform, ie. macOS. Most liked this is solved with:

        > sudo apt-get install libfreetype6


=================================================================================================
##Installation of pyEfis

https://github.com/makerplane/Documentation/blob/master/pyEFIS%20Documentation%20v0-4.pdf
pyEfis works within the MakerPlane Avionics System of Systems and requires the FiX Gateway
to pass data to and from the display application. It also has dependencies in order for the
application to work correctly. MakerPlane currently does not have a single installer however the
following is a step-by-step guide on installation on a linux system (such as the Raspberry Pi or
Beaglebone from scratch). This installation guide assumes the user is connected to the internet
in order to download the required packages.
Step 1: Install PyQT
The primary dependency is PyQt. Installation instructions can be found here:
https://www.riverbankcomputing.com/static/Docs/PyQt5/installation.html
On Debian based distributions apt should work.
$ sudo apt-get install python3-pyqt5
Step 2: Install pip3
You will need pip3 to be installed for the rest of these dependencies.
See https://packaging.python.org/tutorials/installing-packages/ for information on installing pip
and it's associated tools. On Debian based distributions you should be able to simply run:
$ sudo apt install python3-pip
Step 3: Install geomag
Geomag calculates magnetic variation for any latitude/longitude/altitude for any date. It is a
requirement for pyEfis and the FiX Gateway.
$ sudo pip3 install geomag
Step 4: Install FiX Gateway
FIX-Gateway is the backend data gathering application. Currently the best way to install FIX-
Gateway is to download the current archive from the GitHub repository.
$ sudo git clone https://github.com/makerplane/Fix-Gateway.git fixgw
This will copy all of the files and create a directory on your machine called ‘fixgw’.
Next you'll change into the directory that was created and run the setup utility to install the
software.
$ cd fixgw
$ sudo pip3 install .
Now you can run FIX-Gateway with the following command.
$ sudo ./fixgwc.py
Or
$ sudo ./fixgw.py
These will run the client and server respectively.
It may complain about some missing modules but it should still start up. To verify that it is
running correctly you can use the installed client program.
At the FIX> prompt type status and it should show information about the state of the FIX-
Gateway service. You can use the client to read and write data in the data base to change what
pyEfis is displaying. See the FIX-Gateway documentation on the GitHub repository for detailed
information on how to use FIX-Gateway.
https://github.com/makerplane/FIX-Gateway
Step 5: Install pyAvTools
Next we need to install pyAvTools. This is a Python package which contains aviation related
tools and libraries.
The installation is similar to FIX-Gateway and can be done by downloading an
$ git clone https://github.com/makerplane/pyAvTools.git pyAvTools
Next you'll change into the directory that was created and run the setup utility to install the
software.
$ cd PyAvTools
$ sudo pip3 install .
Step 6: Install pyEfis
Now we can finally install pyEfis itself.
$ sudo git clone https://github.com/makerplane/pyEfis.git pyEfis
This will copy all of the files and create a directory on your machine called ‘pyEfis’.
Next you'll change into the directory that was created and run the setup utility to install the
software.
$ cd pyEfis
Now you can run pyEfis with the following command.
$ sudo ./pyEfis.py
If all has worked you should get an EFIS displayed on your desktop. (Make sure you have
launched fixgw.py first.)
Hints and tips on how to configure pyEFIS and the FiX gateway and launch automatically on
startup are in the MakerPlane forum.
http://www.makerplane.org/forum/
end pyEfis
================================================================================================

=======================
##Docker installation
for SITL using gazebo and docker, docker install is kinda lengthy
==========================================================================
https://github.com/jonasvautherin/px4-gazebo-headless#readme
Installation of PX4 SITL using gazebo
Installation of gazebo px4 SITL 
if not using pixhawk autopilot or qgcs forwarded to ip:port
gazebo headless PX4 simulator (SITL)
https://github.com/jonasvautherin/px4-gazebo-headless
The Docker images resulting from this repo are available on Docker Hub.

Note that the following commands are referring to the latest supported release of PX4, which is currently v1.14.0.
Run in BROADCAST mode:

In this mode, the simulator will be available from your host (e.g. run the following command, and QGroundControl running on your computer will connect automatically).

docker run --rm -it jonasvautherin/px4-gazebo-headless:1.14.0

In this configuration, the container will send MAVLink to the host on ports 14550 (for QGC) and 14540 (for e.g. MAVSDK).

https://github.com/jonasvautherin/px4-gazebo-headless#readme
=================================================================================================
installing docker ubuntu mint 22.04
https://linuxiac.com/how-to-install-docker-on-linux-mint-21/
Step 1: Install Prerequisites

First, run the two commands below to update the package index and install the prerequisite necessary to add and use a new HTTPS repository.

$ sudo apt update
$ sudo apt install apt-transport-https ca-certificates curl gnupg

Once operations are completed, you can move to the next section, where we’ll add the Docker’s repo GPG key and repo itself to our Linux Mint 21 system.
Step 2: Add Docker’s Official GPG Key

Next, import the Docker GPG repository key to your Mint system. This security feature ensures that the software you’re installing is authentic.

$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker.gpg
()Add Docker’s repo GPG key.)
Notice that the command produces no output.

Step 3: Add Docker Repo to Linux Mint 21

After importing the GPG keys, we’ll add the official Docker repository to our Linux Mint 21 system. This implies that the update package will be made available with the rest of your system’s regular updates if a new version is released.

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu jammy stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

Add the official Docker repository to Linux Mint 21.
Add the official Docker repository to Linux Mint 21.

As with the previous command, its execution produces no output.

Next, refresh the package list.

sudo apt update

Update the package base.
Update the package base.

This command updates the package index on our system. As you can see, the newly added Docker repository is now available and ready to be used.
Step 4: Install Docker on Linux Mint 21

Finally, run the below command to install the latest up-to-date Docker release on Linux Mint.

sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

Install Docker on Linux Mint 21.
Install Docker on Linux Mint 21.

This installs the following Docker components:

    docker-ce: The Docker engine itself.
    docker-ce-cli: A command line tool that lets you talk to the Docker daemon.
    containerd.io: A container runtime that manages the container’s lifecycle.
    docker-buildx-plugin: A CLI plugin that extends the Docker build with many new features.
    docker-compose-plugin: A configuration management plugin to orchestrate creating and managing Docker containers through compose files.

That’s all! Docker should now be installed; the service started and enabled to start automatically on boot.

In addition, you can check the Docker service status using the following:

sudo systemctl is-active docker

Check the status of the Docker service.
Check the status of the Docker service.
Step 5: Verify the Docker Installation

Now let’s check if everything with our new Docker installation works properly. For this purpose, we will run a simple application called “hello-world.”

sudo docker run hello-world

Docker successfully installed, up & running on Linux Mint 21.
Docker successfully installed, up & running on Linux Mint 21.

Congratulations! As we can see, everything works as expected!
==============================================
Enabling Non-root Users to Run Docker Commands

So far, we have successfully installed Docker on your Linux Mint 21 system.

However, only root and users with sudo privileges can execute Docker commands by default, which can be a security concern. In other words, if you attempt to run the docker command as a regular user without prefixing it with sudo, you’ll get an error message like this:
Docker permission denied.
Docker permission denied.

==============================================
So, to run Docker commands as a non-root user, you must add your user to the docker group. To do that, type in the following:

$ sudo usermod -aG docker ${USER}

In the above command, ${USER} is an environment variable that holds your username. To apply for the new group membership, reboot your Mint system. You can then execute docker commands without prefixing them with sudo.
Run the docker command as a regular user.
Run the docker command as a regular user.

This approach can reduce the security risks associated with Docker while enabling non-root users to take advantage of its powerful capabilities.
Conclusion

Installing Docker on Linux Mint 21 can initially seem daunting, but with the step-by-step guide provided, it is a relatively straightforward process. Following the steps outlined in the guide, you can successfully install Docker on your Linux Mint 21 machine and start reaping the benefits of containerization.

So, what are you waiting for? Start experimenting with Docker today and see how it can revolutionize how you build and deploy applications.

Let me know if you have any questions or suggestions, and I’ll be happy to follow up with you. Happy dockering!

To learn more about Docker, check out the official Docker documentation.
Bobby Borisov
Bobby Borisov

Bobby, an editor-in-chief at Linuxiac, is a Linux professional with over 20 years of experience. With a strong focus on Linux and open-source software, he has worked as a Senior Linux System Administrator, Software Developer, and DevOps Engineer for small and large multinational companies.

Think You're an Ubuntu Expert? Let's Find Out!

Put your knowledge to the test in our lightning-fast Ubuntu quiz!
Ten questions to challenge yourself to see if you're a Linux legend or just a penguin in the making.
POPULAR
GNOME's Giant Leap: Open-Source Project Receives €1 Million Boost
GNOME’s Giant Leap: Open-Source Project Receives €1 Million Boost
Fedora 39 Released, It Is All about GNOME 45
Fedora 39 Released, It Is All about GNOME 45
3 Best Linux Terminal Editors Perfect for Beginners
3 Best Linux Terminal Text Editors Perfect for Beginners
How To Protect SSH with Fail2Ban: A Beginner's Guide
How To Protect SSH with Fail2Ban: A Beginner’s Guide
Ubuntu 23.10
Ubuntu 23.10 “Mantic Minotaur” Released, Here’s What’s New
Subscribe to our newsletter!

