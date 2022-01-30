#!/bin/bash

# This is an example bash file for build and run ChronoGpu projects
# Update the numbers of compute nodes, cpu cores and gpus (-N, -n, --gres=gpu:) and computing wall time (-t) based on your computation needs
# Update your email address (--mail-user=) to recieve notifications
# replace all the instances of "mychgpu" with your project title (such as "sample_generation")


#SBATCH -N 1                        	# number of compute nodes
#SBATCH -n 4                        	# number of tasks your job will spawn
#SBATCH --mem=16G                   	# amount of RAM requested in GiB (2^40)
#SBATCH -p gpu                     	  # Use gpu partition
#SBATCH -C V100                     	# Specify GPU type
#SBATCH -q wildfire                 	# Run job under wildfire QOS queue
#SBATCH --gres=gpu:4                	# Request 4 GPUs
#SBATCH -t 2-00:00                  	# wall time (D-HH:MM)
#SBATCH -o slurm.%j.out             	# STDOUT (%j = JobId)
#SBATCH -e slurm.%j.err             	# STDERR (%j = JobId)
#SBATCH --mail-type=ALL             	# Send a notification when a job starts, stops, or fails
#SBATCH --mail-user=jtao25@asu.edu 	  # send-to email address, change to your own when using
module purge
module load eigen/3.3.7-openmpi-3.0.3-gcc-7x cuda/11.2.0 blaze/3.7 cmake/3.20.3 chrono/6.0.0 anaconda/py3 rclone/1.43
nvidia-smi  # Useful for seeing GPU status and activity
mkdir build # Make a new directory to host all cmake files
cd build    # Go to the new build directory
cmake ..    # Generate Makefiles
make        # Build the project
srun --mpi=pmi2 ./mychgpu mychgpu.json			# Run the executive

########################NOTE########################
# The following is for data archiving purposes     #
# If you have not setup rclone ang GoogleDrive yet,# 
# you may want to remove the following lines       #
####################################################

cd ..                             # Return to the project directory
tar czvf mychgpu_output.tgz ./OUT # Make a tarball of the output files with compression
# Move output files to GoogleDrive shared folder. BiGdata is the shared group folder
# BiGdata:mychgpu will create a new subfolder with the project name "mychgpu"
rclone copy ./mychgpu_output.tgz BiGdata:mychgpu 
rm -rf ./OUT mychgpu_output.tgz   # Remove output files
