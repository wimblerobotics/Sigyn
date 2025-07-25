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
alias cb='colcon build --symlink-install'
alias cgcm='ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap'
alias clcm='ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap'
alias dla="ros2 run --prefix 'gdbserver localhost:3000' line_finder laser_accumulator"
alias fr='ros2 run tf2_tools view_frames'
alias gripper='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_gripper'
alias map='clear;ros2 launch base sigyn.launch.py use_sim_time:=false do_rviz:=true make_map:=true'
alias nav='clear;ros2 launch base sigyn.launch.py use_sim_time:=false do_rviz:=true'
alias pm='ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"'
alias rd='rosdep install --from-paths src --ignore-src -r -y'
alias record='/home/ros/sigyn_ws/src/Sigyn/scripts/bag_record_sim.sh'
alias redoudev='sudo service udev restart;sudo udevadm control --reload-rules;sudo udevadm trigger'
alias rms='ros2 launch nav2_map_server map_saver_server.launch.py'
alias rvs='rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz'
alias savem='ros2 run nav2_map_server map_saver_cli -f my_map'
alias sim='clear;ros2 launch base sigyn.launch.py use_sim_time:=true do_rviz:=true'
alias sp='ssh -YC signpi'
alias sr='ssh -YC sigyn7900'
alias stele='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_teleop'
alias teensy='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_sensor'
alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias toggle='ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'

alias compileBoard1='platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2'
alias compileBoard2='platformio run -e board2 -d ~/sigyn_ws/src/Sigyn/TeensyV2'
alias buildBoard1='platformio run -e board1 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload'
alias buildBoard2='platformio run -e board2 -d ~/sigyn_ws/src/Sigyn/TeensyV2 --target upload'


#export CYCLONEDDS_URI="
#<CycloneDDS>
#   <Domain>
#     <General>
#        <Interfaces>
#          <NetworkInterface name='wlp38s0' />
#        </Interfaces>
#    </General>
#   </Domain>
#</CycloneDDS>"

#export ROS_DOMAIN_ID=0
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export PATH=$PATH:~/.local/bin

source /opt/ros/jazzy/setup.bash
source ~/sigyn_ws/install/setup.bash
source ~/sigyn_microros_ws/install/setup.bash
