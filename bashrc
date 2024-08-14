# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

alias arduino='/home/ros/arduino-1.8.19/arduino&'
alias cb='colcon build --symlink-install'
alias fr='ros2 run tf2_tools view_frames'
alias nav='clear;ros2 launch base navigation.launch.py'
alias pm='ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'
#alias record='/home/ros/raven_ws/src/raven/raven_base/scripts/rosbag_record.sh'
alias rd='rosdep install --from-paths src --ignore-src -r -y'
#alias record='rm -r ~/bag;sh /home/ros/raven_ws/install/raven_base/share/raven_base/scripts/rosbag_record.sh'
#alias roam='DO_RVIZ=true DO_JOYSTICK=false DO_NAV2=true ros2 launch raven_base roam.launch.py'
#alias roam_nrviz='DO_RVIZ=false DO_JOYSTICK=false DO_NAV2=true ros2 launch raven_base roam.launch.py'
#alias roam_joy='DO_RVIZ=true DO_JOYSTICK=true DO_NAV2=true ros2 launch raven_base roam_launch.py'
alias rvs='rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz'
alias save_map='ros2 run nav2_map_server map_saver_cli -f ~/map_`date +"%Y_%m_%d_%H%M"`'
#alias save_map='ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: src/s/base/maps/mapx, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"'
#alias s2='source /home/ros/setupRos2.sh'
alias sim='clear;ros2 launch base sim.launch.py'
alias teensy='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_sensor'
alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias toggle='ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'
alias redoudev='sudo service udev restart;sudo udevadm control --reload-rules;sudo udevadm trigger'
alias dla="ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator"

# Entry point for Depthai demo app, enables to run <depthai_launcher> in terminal
export PATH=$PATH:/home/ros/Luxonis/depthai/entrypoint
export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=ldlidar
export LINOROBOT2_DEPTH_SENSOR=realsense
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/microros_humble_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
source ~/sigyn_ws/install/setup.bash

export CYCLONEDDS_URI="
<CycloneDDS>
   <Domain>
     <General>
        <Interfaces>
          <NetworkInterface name='wlp8s0' />
        </Interfaces>
    </General>
   </Domain>
</CycloneDDS>"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
