## Welcome to GitHub Pages

You can use the [editor on GitHub](https://github.com/SaraTohidi/traversal_map/edit/master/README.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3
Usage

We assume you have an Ubuntu 16.04 machine set up. Follow these steps to bring up the robot:

Installing Ros Kinetic

First you have to install ROS kinetic:

Set up your sources.list

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
Set up your keys

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
Installation

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
Installing Gazebo

For using 3D simulated environment we use Gazebo7:

sudo apt-get install gazebo7
Install Some ROS Packages

This packages are needed for the code to be maked and compiled:

Controller Manager

sudo apt-get install ros-kinetic-controller-manager 
Joy

sudo apt-get install ros-kinetic-joy
Gmapping

sudo apt-get install ros-kinetic-gmapping
Move Base

sudo apt-get install ros-kinetic-move-base
Hector Mapping

sudo apt-get install ros-kinetic-hector-mapping
Compiling and Running

Here we should first create a catkin workspace and compile and run our code. Follow these steps to finally see the result:

Creating a workspace

Use these commands to create a catkin workspace: (here we name it catkin_ws)

source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
Then copy the contents of src folder from here to the src folder of your workspace.

Compiling the code

Assuming you're in the home directory and your catkin workspace folder is named catkin_ws use these commands:

cd catkin_ws
catkin_make
Source your project shell environment variables :

Run this command before run each

source ./devel/setup.bash
or (if using zsh shell )

source ./devel/setup.zsh
- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/SaraTohidi/traversal_map/settings). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://help.github.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.
