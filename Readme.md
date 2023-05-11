# 18.337 FINAL PROJECT: PERCEPTION-AWARE MULTIAGENT TRAJECTORY PLANNER USING IMITATION LEARNING #

<p align="center">
<img src="./panther/imgs/3-agent-2-obsts-primer.gif" width="700">
</p>

We extended our prior work Deep-PANTHER, which is a trajectory planner for a single agent with single obstacles. In this project, we extend it to a multi-agent planner that can handle multiple obstacles. We also developed an imitation learning-based approach that can generate both position and yaw trajectories, whereas Deep-PANTHER only produced position trajectories.

## Usage

Simply use:
```bash
roslaunch panther simulation.launch

```

Wait until the terminal says `Planner initialized`. Then, you can press G (or click the option 2D Nav Goal on the top bar of RVIZ) and click any goal for the drone. By default, `simulation.launch` will use the policy Hung_dynamic_obstacles.pt (which was trained with trefoil-knot trajectories). You can change the trajectory followed by the obstacle during testing using the `type_of_obst_traj` field of the launch file.

## Julia MPI Training

You first need to install a linear solver (see instructions below). 
Then, you can train a new policy by simply running `main.jl` inside the `panther/julia` folder. If you would like to specify the number of processer, you can use `-np` flag.


## General Setup

Our planner has been tested with Ubuntu 20.04/ROS Noetic.

The instructions below assume that you have ROS Noetic installed on your Linux machine.

### <ins>Dependencies<ins>

> Note: the instructions below are partly taken from [here](https://github.com/casadi/casadi/wiki/InstallationLinux#installation-on-linux)

#### IPOPT
```bash
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
sudo apt-get install coinor-libipopt1v5 coinor-libipopt-dev
```

#### CasADi
```bash
sudo apt-get remove swig swig3.0 swig4.0 #If you don't do this, the compilation of casadi may fail with the error "swig error : Unrecognized option -matlab"
mkdir ~/installations && cd ~/installations
git clone https://github.com/jaeandersson/swig
cd swig
git checkout -b matlab-customdoc origin/matlab-customdoc        
sh autogen.sh
sudo apt-get install gcc-7 g++-7 bison byacc
./configure CXX=g++-7 CC=gcc-7            
make
sudo make install


cd ~/installations && mkdir casadi && cd casadi
git clone https://github.com/casadi/casadi
cd casadi 
#cd build && make clean && cd .. && rm -rf build #Only if you want to clean any previous installation/compilation 
mkdir build && cd build
cmake . -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON -DWITH_MATLAB=OFF -DWITH_PYTHON=ON -DWITH_DEEPBIND=ON ..
#You may need to run the command above twice until the output says that `Ipopt` has been detected (although `IPOPT` is also being detected when you run it for the first time)
make -j20
sudo make install
```
#### Virtual Python environment
```bash
sudo apt-get install python3-venv
cd ~/installations && mkdir venvs_python && cd venvs_python 
python3 -m venv ./my_venv
printf '\nalias activate_my_venv="source ~/installations/venvs_python/my_venv/bin/activate"' >> ~/.bashrc
source ~/.bashrc
activate_my_venv
```

### <ins>Compilation<ins>
And finally download the repo and compile it:

```bash
sudo apt-get install git-lfs ccache 
cd ~/Desktop/
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/kotakondo/18337.git
cd 18337
git lfs install
git submodule init && git submodule update
cd panther_compression/imitation
pip install numpy Cython wheel seals rospkg defusedxml empy pyquaternion pytest
pip install -e .
sudo apt-get install python3-catkin-tools #To use catkin build
sudo apt-get install ros-"${ROS_DISTRO}"-rviz-visual-tools ros-"${ROS_DISTRO}"-pybind11-catkin ros-"${ROS_DISTRO}"-tf2-sensor-msgs ros-"${ROS_DISTRO}"-jsk-rviz-plugins
cd ~/Desktop/ws/
catkin build
printf '\nsource PATH_TO_YOUR_WS/devel/setup.bash' >> ~/.bashrc #Remember to change PATH_TO_YOUR_WS
printf '\nexport PYTHONPATH="${PYTHONPATH}:$(rospack find panther)/../panther_compression"' >> ~/.bashrc 
source ~/.bashrc
```

<details>
  <summary> <b>MATLAB (optional dependency)</b></summary>

First, when installing CasADi following the instructions above, you need to use `-DWITH_MATLAB=ON` instead of `-DWITH_MATLAB=OFF`. Then do the following:

```bash
#Open MATLAB, and type this:
edit(fullfile(userpath,'startup.m'))
#And in that file, add this line line 
addpath(genpath('/usr/local/matlab/'))
```

Now, you can restart Matlab (or run the file `startup.m`), and make sure this works:

```bash
import casadi.*
x = MX.sym('x')
disp(jacobian(sin(x),x))
```

</details>

<details>
  <summary> <b>Linear Solver (optional dependency)</b></summary>

Go to [http://www.hsl.rl.ac.uk/ipopt/](http://www.hsl.rl.ac.uk/ipopt/), click on `Personal Licence, Source` to install the solver `MA27` (free for everyone), and fill and submit the form. Once you receive the corresponding email, download the compressed file, uncompress it, and place it in the folder `~/installations` (for example). Then execute the following commands:

> Note: the instructions below follow [this](https://github.com/casadi/casadi/wiki/Obtaining-HSL) closely

```bash
cd ~/installations/coinhsl-2015.06.23
wget http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/OLD/metis-4.0.3.tar.gz #This is the metis version used in the configure file of coinhsl
tar xvzf metis-4.0.3.tar.gz
#sudo make uninstall && sudo make clean #Only needed if you have installed it before
./configure LIBS="-llapack" --with-blas="-L/usr/lib -lblas" CXXFLAGS="-g -O3 -fopenmp" FCFLAGS="-g -O3 -fopenmp" CFLAGS="-g -O3 -fopenmp" #the output should say `checking for metis to compile... yes`
sudo make install #(the files will go to /usr/local/lib)
cd /usr/local/lib
sudo ln -s libcoinhsl.so libhsl.so #(This creates a symbolic link `libhsl.so` pointing to `libcoinhsl.so`). See https://github.com/casadi/casadi/issues/1437
echo "export LD_LIBRARY_PATH='\${LD_LIBRARY_PATH}:/usr/local/lib'" >> ~/.bashrc
```

<details>
  <summary> <b>Note</b></summary>

We recommend to use `MA27`. Alternatively, you can install both `MA27` and `MA57` by clicking on `Coin-HSL Full (Stable) Source` (free for academia) in [http://www.hsl.rl.ac.uk/ipopt/](http://www.hsl.rl.ac.uk/ipopt/) and then following the instructions above. Other alternative is to use the default `mumps` solver (no additional installation required), but its much slower than `MA27` or `MA57` You can change the linear solver used by changing the name of `linear_solver_name` in the file `main.m` and run that file.

Moreover, when using a linear solver different from `mumps`, you may need to start Matlab from the terminal (typing `matlab`). More info [in this issue](https://github.com/casadi/casadi/issues/2032). 

</details>

</details>
