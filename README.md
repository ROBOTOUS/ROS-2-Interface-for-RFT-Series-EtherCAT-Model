# ROS 2 Interface Guide for RFT Series (EtherCAT Model)


This guide provides instructions for interfacing the **RFT sensor**, which uses **EtherCAT communication**, within a **ROS 2** environment.

Steps 2 through 4 require a total of **three terminals**.
Each step must be executed in a **separate terminal**.


---

## System Requirements

* **ROS 2** must be installed
* Verified in a **virtual environment** using **Ubuntu 20.04** with **ROS 2 Foxy**

---



## 1. ROS 2 Workspace Setup

---

### 1-1. Create your workspace

Create a directory that will contain your ROS 2 workspace.

```bash
mkdir -p ~/ {your workspace}
cd ~/ {your workspace}
```

---

### 1-2. Create the 'src' folder

The 'src' folder is not generated automaticaly. You need to create it manually to store ROS 2 packages.

```bash
mkdir src
```

---

### 1-3. Unzip your ROS 2 package into the 'src' folder

Copy 'rft_ethercat_ros2_interface.zip' into 'src' and extract it.

```bash
cp ~/downloads/rft_ethercat_ros2_interface.zip.zip ~/{your_workspace}/src/
cd ~/{rft_ethercat_ros2_interface.zip}/src
unzip rft_ethercat_ros2_interface.zip.zip
```

---

### 1-4. Build the workspace

Build all packages in the 'src' folder using 'colcon'.

```bash
cd ~/{your workspace}
colcon build
```

After the build completes, verify that the following directories are created:

* build/

* install/

* log/

---

## 2. EtherCAT Environment Setup and Interface Execution

This section describes the complete process required to install dependencies, configure permissions, and run the EtherCAT interface

---

### 2-1. Install EtherCAT Python Dependency (pysoem)

Create a directory that will contain your ROS 2 workspace.

```bash
sudo pip3 install pysoem
```

Root privileges are required because **'EtherCAT**' uses **'raw network sockets**'.
Setting network capabilities **'cap_net_raw**' also requires **'sudo**'.

---

### 2-2. Configure Python Permissions for EtherCAT

Check the Python version in use and grant raw network socket permission.

```bash
cd /usr/bin
ls python3.8
sudo setcap cap_net_raw+ep /usr/bin/python3.8
```

Applying setcap restricts environment variables.
This may prevent ROS 2 shared libraries from loading correctly.
Therefore, the interface must be executed as root.
A permanent non-root solution is under investigation.

---

### 2-3. Run the EtherCAT Interface Node

Switch to the root user

```bash
sudo su
```

Navigate to the workspace and source the ROS 2 environment

```bash
cd /home/{username}/{your_workspace}
source /opt/ros/<your_ros2_distro>/setup.bash
source ./install/setup.bash
```

Move to the script directory and run the interface

```bash
cd ~/{your_workspace}/src/rft_ethercat_ros2_interface/script
python3 interface.py 

```

Find the network adapter name using

```bash
ifconfig
```

Identify the active network adapter used for EtherCAT communication (for example: enp0s3).
In the script directory, open the config.txt file and enter the identified adapter name as shown below
Replace enp0s3 with the actual adapter name detected on your system.

---

## For a detailed usage guide, please refer to the provided PDF documentation.
