#!/bin/bash
# Ensure that you have installed docker(API >= 1.40) and the nvidia graphics driver on host!
# Copyright 2018-2022, Kolin Guo, All rights reserved.

# Created by argbash-init v2.10.0
# Rearrange the order of options below according to what you would like to see in the help message.
# ARG_OPTIONAL_BOOLEAN([build-local],[l],[build docker image locally or pull from DockerHub],[off])
# ARG_OPTIONAL_BOOLEAN([rm-img],[],[remove previously built docker image],[off])
# ARG_OPTIONAL_BOOLEAN([bashrc-only],[],[generate only the custom bashrc],[off])
# ARGBASH_SET_INDENT([  ])
# ARGBASH_SET_DELIM([ =])
# ARG_OPTION_STACKING([getopt])
# ARG_RESTRICT_VALUES([no-local-options])
# ARG_HELP([Generic docker image/container setup script, adapted for maya-slam],[Copyright 2018-2022, Kolin Guo, All rights reserved.\n])
# ARG_VERSION_AUTO([v0.1.0],['Copyright 2018-2022, Kolin Guo, All rights reserved.'],[V])
# ARGBASH_GO()
# needed because of Argbash --> m4_ignore([
### START OF CODE GENERATED BY Argbash v2.10.0 one line above ###
# Argbash is a bash code generator used to get arguments parsing right.
# Argbash is FREE SOFTWARE, see https://argbash.io for more info


die()
{
  local _ret="${2:-1}"
  test "${_PRINT_HELP:-no}" = yes && print_help >&2
  echo "$1" >&2
  exit "${_ret}"
}


evaluate_strictness()
{
  [[ "$2" =~ ^-(-(build-local|rm-img|bashrc-only|help|version)$|[lhV]) ]] && die "You have passed '$2' as a value of argument '$1', which makes it look like that you have omitted the actual value, since '$2' is an option accepted by this script. This is considered a fatal error."
}


begins_with_short_option()
{
  local first_option all_short_options='lhV'
  first_option="${1:0:1}"
  test "$all_short_options" = "${all_short_options/$first_option/}" && return 1 || return 0
}

# THE DEFAULTS INITIALIZATION - OPTIONALS
_arg_build_local="off"
_arg_rm_img="off"
_arg_bashrc_only="off"


print_help()
{
  local usage="Usage: ${0} [OPTION]...\n"
  usage+="Generic docker image/container setup script, adapted for maya-slam\n\n"
  usage+="  -l, --build-local       build docker image locally with Dockerfile\n"
  usage+="      --no-build-local    pull image from DockerHub (default)\n"
  usage+="  --rm-img                remove previously built docker image\n"
  usage+="      --no-rm-img         (off by default)\n"
  usage+="  --bashrc-only           generate only the custom bashrc\n"
  usage+="      --no-bashrc-only    (off by default)\n"
  usage+="      -h, --help     display this help and exit\n"
  usage+="      -V, --version  output version information and exit\n"
  usage+="\nCopyright 2018-2022, Kolin Guo, All rights reserved.\n"

  echo -e "$usage"
}


parse_commandline()
{
  while test $# -gt 0
  do
    _key="$1"
    case "$_key" in
      -l|--no-build-local|--build-local)
        _arg_build_local="on"
        test "${1:0:5}" = "--no-" && _arg_build_local="off"
        ;;
      -l*)
        _arg_build_local="on"
        _next="${_key##-l}"
        if test -n "$_next" -a "$_next" != "$_key"
        then
          { begins_with_short_option "$_next" && shift && set -- "-l" "-${_next}" "$@"; } || die "The short option '$_key' can't be decomposed to ${_key:0:2} and -${_key:2}, because ${_key:0:2} doesn't accept value and '-${_key:2:1}' doesn't correspond to a short option."
        fi
        ;;
      --no-rm-img|--rm-img)
        _arg_rm_img="on"
        test "${1:0:5}" = "--no-" && _arg_rm_img="off"
        ;;
      --no-bashrc-only|--bashrc-only)
        _arg_bashrc_only="on"
        test "${1:0:5}" = "--no-" && _arg_bashrc_only="off"
        ;;
      -h|--help)
        print_help
        exit 0
        ;;
      -h*)
        print_help
        exit 0
        ;;
      -V|--version)
        printf '%s %s\n\n%s\n%s\n' "docker_setup.sh" "v0.1.0" 'Generic docker image/container setup script, adapted for maya-slam' 'Copyright 2018-2022, Kolin Guo, All rights reserved.'
        exit 0
        ;;
      -V*)
        printf '%s %s\n\n%s\n%s\n' "docker_setup.sh" "v0.1.0" 'Generic docker image/container setup script, adapted for maya-slam' 'Copyright 2018-2022, Kolin Guo, All rights reserved.'
        exit 0
        ;;
      *)
        _PRINT_HELP=yes die "FATAL ERROR: Got an unexpected argument '$1'" 1
        ;;
    esac
    shift
  done
}

parse_commandline "$@"

# OTHER STUFF GENERATED BY Argbash

### END OF CODE GENERATED BY Argbash (sortof) ### ])
# [ <-- needed because of Argbash

### Save the nice-looking usage ###
#print_help()
#{
#  local usage="Usage: ${0} [OPTION]...\n"
#  usage+="Generic docker image/container setup script, adapted for maya-slam\n\n"
#  usage+="  -l, --build-local       build docker image locally with Dockerfile\n"
#  usage+="      --no-build-local    pull image from DockerHub (default)\n"
#  usage+="  --rm-img                remove previously built docker image\n"
#  usage+="      --no-rm-img         (off by default)\n"
#  usage+="  --bashrc-only           generate only the custom bashrc\n"
#  usage+="      --no-bashrc-only    (off by default)\n"
#  usage+="      -h, --help     display this help and exit\n"
#  usage+="      -V, --version  output version information and exit\n"
#  usage+="\nCopyright 2018-2022, Kolin Guo, All rights reserved.\n"
#
#  echo -e "$usage"
#}

# vvv  PLACE YOUR CODE HERE  vvv
############################################################
# Section 0: Project-Specific Settings                     #
############################################################
IMGNAME="kolinguo/maplab"
CONTNAME="maplab"
DOCKERFILEPATH="./docker/Dockerfile_maplab"
REPONAME="maya-slam"
JUPYTERPORT="9000"
TENSORBOARDPORT="6007"

COMMANDTOCOMPILE="cd slam_algorithms/maplab && ./build_ros.sh"
COMMANDTORUNJUPYTER="jupyter notebook --no-browser --ip=0.0.0.0 --allow-root --port=${JUPYTERPORT} &"
COMMANDTORUNTENSORBOARD="tensorboard --logdir /${REPONAME}/tb_logs/ --port ${TENSORBOARDPORT} --host 0.0.0.0 >/dev/null 2>&1 &"
COMMANDTOSTARTCONTAINER="docker start -ai ${CONTNAME}"

############################################################
# Section 1: Helper Function Definition                    #
############################################################
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

test_retval() {
  if [ $? -ne 0 ] ; then
    echo -e "\nFailed to ${*}... Exiting...\n"
    exit 1
  fi
}

print_setup_info() {
  # Echo the set up information
  echo -e "\n\n"
  echo -e "################################################################################\n"
  echo -e "\tSet Up Information\n"
  if [ "$_arg_bashrc_only" = "on" ] ; then
    echo -e "\t\tOnly generate the custom bashrc\n"
  fi
  if [ "$_arg_rm_img" = "on" ] ; then
    echo -e "\t\tCautious!! Remove previously built Docker image\n"
  else
    echo -e "\t\tKeep previously built Docker image\n"
  fi
  echo -e "################################################################################\n"
}

remove_prev_docker_image () {
  # Remove previously built Docker image
  if [ "$_arg_rm_img" = "on" ] ; then
    echo -e "\nRemoving previously built image..."
    docker rmi -f $IMGNAME
  fi
}

create_custom_bashrc() {
  cat > bashrc <<- "EOF"
# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# enable bash completion in interactive shells
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

. /etc/bash_completion

# if the command-not-found package is installed, use it
if [ -x /usr/lib/command-not-found -o -x /usr/share/command-not-found/command-not-found ]; then
  function command_not_found_handle {
    # check because c-n-f could've been removed in the meantime
    if [ -x /usr/lib/command-not-found ]; then
      /usr/lib/command-not-found -- "$1"
      return $?
    elif [ -x /usr/share/command-not-found/command-not-found ]; then
      /usr/share/command-not-found/command-not-found -- "$1"
      return $?
    else
      printf "%s: command not found\n" "$1" >&2
      return 127
    fi
  }
fi

# Change PS1 and terminal color
export PS1="\[\e[31m\]${CONTNAME}-docker\[\e[m\] \[\e[33m\]\w\[\e[m\] > "  # match square bracket for argbash: ]]]]
export TERM=xterm-256color
alias grep="grep --color=auto"
alias ls="ls --color=auto"

# some more ls aliases
alias ll="ls -alF"
alias la="ls -A"
alias l="ls -CF"

# Cyan color
echo -e "\e[1;36m"  # match square bracket for argbash: ]

# >>> conda initialize >>>
export CONDA_AUTO_ACTIVATE_BASE=false
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/miniconda/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/miniconda/etc/profile.d/conda.sh" ]; then
        . "/miniconda/etc/profile.d/conda.sh"
    else
        export PATH="$PATH:/miniconda/bin"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

EOF

  # Overwrite CONTNAME
  sed -i "s/\${CONTNAME}/${CONTNAME}/" bashrc

  # Echo command to compile
  if [ ! -z "$COMMANDTOCOMPILE" ] ; then
    echo -n COMMANDTOCOMPILE=\" >> bashrc \
      && echo -n ${COMMANDTOCOMPILE} | sed 's/\"/\\"/g' >> bashrc \
      && echo \" >> bashrc \
      && echo >> bashrc
  fi

  # Echo command to run jupyter notebook
  if [ ! -z "$COMMANDTORUNJUPYTER" ] ; then
    echo -n COMMANDTORUNJUPYTER=\" >> bashrc \
      && echo -n ${COMMANDTORUNJUPYTER} | sed 's/\"/\\"/g' >> bashrc \
      && echo \" >> bashrc \
      && echo >> bashrc
  fi

  # Echo command to run tensorboard
  if [ ! -z "$COMMANDTORUNTENSORBOARD" ] ; then
    echo -n COMMANDTORUNTENSORBOARD=\" >> bashrc \
      && echo -n ${COMMANDTORUNTENSORBOARD} | sed 's/\"/\\"/g' >> bashrc \
      && echo \" >> bashrc \
      && echo >> bashrc
  fi

  # Echo the echo command to print instructions
  echo echo -e \"\" >> bashrc
  echo echo -e \"################################################################################\\n\" >> bashrc
  if [ ! -z "$COMMANDTOCOMPILE" ] ; then
    echo echo -e \"\\tCommand to compile:\\n\\t\\t'${COMMANDTOCOMPILE}'\\n\" >> bashrc
  fi
  if [ ! -z "$COMMANDTORUNJUPYTER" ] ; then
    echo echo -e \"\\tCommand to run jupyter notebook:\\n\\t\\t'${COMMANDTORUNJUPYTER}'\\n\" >> bashrc
  fi
  if [ ! -z "$COMMANDTORUNTENSORBOARD" ] ; then
    echo echo -e \"\\tCommand to run TensorBoard:\\n\\t\\t'${COMMANDTORUNTENSORBOARD}'\\n\" >> bashrc
  fi
  echo echo -e \"################################################################################\\n\" >> bashrc \
    && echo >> bashrc

  # Change terminal color back
  echo "# Turn off colors" >> bashrc \
    && echo echo -e \"\\e[m\" >> bashrc  # match square bracket for argbash: ]
}

build_docker_image() {
  # Set REPOPATH for WORKDIR
  sed -i "s/^ENV REPOPATH.*$/ENV REPOPATH \/${REPONAME}/" $DOCKERFILEPATH

  # Build and run the image
  echo -e "\nBuilding image $IMGNAME..."
  docker build -f $DOCKERFILEPATH -t $IMGNAME .
  test_retval "build Docker image $IMGNAME"
  rm -rf bashrc
}

build_docker_container() {
  # Build a container from the image
  echo -e "\nRemoving older container $CONTNAME..."
  if [ 1 -eq $(docker container ls -a | grep "$CONTNAME$" | wc -l) ] ; then
    docker rm -f $CONTNAME
  fi

  echo -e "\nBuilding a container $CONTNAME from the image $IMGNAME..."
  docker create -it --name=$CONTNAME \
    -v "$SCRIPTPATH":/$REPONAME \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --ipc=host \
    --gpus all \
    --net=host \
    -p $JUPYTERPORT:$JUPYTERPORT \
    -p $TENSORBOARDPORT:$TENSORBOARDPORT \
    --privileged=true \
    --cap-add=CAP_SYS_ADMIN \
    $IMGNAME /bin/bash
  test_retval "create Docker container"
}

start_docker_container() {
  docker start -ai $CONTNAME

  if [ 0 -eq $(docker container ls -a | grep "$CONTNAME$" | wc -l) ] ; then
    echo -e "\nFailed to start/attach Docker container... Exiting...\n"
    exit 1
  fi
}

print_exit_command() {
  # Echo command to start container
  echo -e "\n"
  echo -e "################################################################################\n"
  echo -e "\tCommand to start Docker container:\n\t\t${COMMANDTOSTARTCONTAINER}\n"
  echo -e "################################################################################\n"
}


############################################################
# Section 2: Call Helper Functions                         #
############################################################
# Print the setup info
print_setup_info
# Print usage of the script
print_help

echo -e ".......... Set up will start in 5 seconds .........."
sleep 5

if [ "$_arg_bashrc_only" = "on" ] ; then
  create_custom_bashrc
  exit 0
fi

remove_prev_docker_image

### Build docker image ###
if [ "$_arg_build_local" = "on" ] ; then
  create_custom_bashrc
  build_docker_image
else
  docker pull $IMGNAME
fi

build_docker_container
start_docker_container

# When exit from docker container
print_exit_command


# ^^^  TERMINATE YOUR CODE BEFORE THE BOTTOM ARGBASH MARKER  ^^^

# ] <-- needed because of Argbash
