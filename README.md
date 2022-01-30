# A template ChronoGPU project to run on Agave

## To test run the template project on Agave

**Warning**: in order to use the google drive and git as instructed, you need to update your personal configure files for `rclone` and `git` on the **cluster**  first. 

If you prefer to use GUI tools for `git` operations, neglect the corresponding command line instructions below and go ahead and use the GUI tools you are familiar with. Do note that using the command line could potentially streamline your workflow and improve your efficiency further.

See the workflow website for detailed information.

1. Logon to the Agave cluster

```bash
ssh your.username@asu.agave.edu
```

2. Navigate to your workspace

   For example,

```bash
cd Workspace/Chrono
```

**Note**: Your working directory may be different from mine. It is recommended to have a directory called something like "Workspace" in your home directory to host all your projects

3. Clone this repo

```bash
git clone https://github.com/JulianTao/chgpu_agave_template.git
```

4. Navigate to the new directory

```bash
cd chgpu_agave_template
```
5. Examine the directory structure

```bash
ls -al
```
   It includes:

   * a CMakeList.txt
   * a bash file to be run by `sbatch`
   * a .gitignore file to ignore the output and log files for git operations
   * source files 
     * mychgpu.cpp
     * GpuDemoUtils.hpp
     * mychgpu.json
6. Edit `mychgpu.sh` if needed, following the instructions inside

8. Run the bash file

```bash
sbatch mychgpu.sh
```
8. Examine the `OUTPUT` directory and the log files `.err` and `.out`.
9. Examine the shared google drive, there should be a new folder data folder with the name specified in the bash script. 

## To create your own project based on the template 

**Warning**: in order to use git as instructed below, you need to configure `git` first **on your computer**. See the workflow website for detailed information.
1. On GitHub, navigate to the main page of the repository. [chgpu_agave_template](https://github.com/JulianTao/chgpu_agave_template) 
2. Above the file list, click `Use this template`, which usually appears in green.
4. Type a name for your repository, and an optional description. The project name should be concise but indicative, e.g., "sample_prep". For a more comprehensive instruction on how to create a repo from a template, check [here](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template) 
**Note**: instead of using "template", you can also "fork" the repo and then update the settings such as repo name etc. There are some differences though, as discussed [here](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template) 

6. **On your own computer**, clone the new repo. 

   **Note that if you want to keep the project names and file names, neglect the renaming and searching/replacing processes in the following steps. This way, you only need to modify the technical contents and it may save some time. The downside is when you have multiple projects, all of the executives will have the same name. It may cause confusions in the long run.**

7. If you prefer, rename the `cpp`,`json`, `sh` files with your project name. 
   For example, `mychgpu.cpp` --> `sample_prep.cpp`; `mychgpu.json` --> `sample_prep.json`; and `mychgpu.sh` --> `sample_prep.json`
8. Edit the source `cpp` and `json` files for your project. Add `.obj` files to the `data` subfolder as needed.
9. Update the `CMakeList.txt` file:

   Replace the word `mychgpu` with your project name.

   For example, change

```CMake
set(MY_PROJECT mychgpu)
```
to

```CMake
set(MY_PROJECT sample_prep)
```
7. Update the `mychgpu.sh` file

  * Search and replace all instances of `mychgpu` with your project name, e.g., `sample_prep`
  * Update the slurm requests as shown in the example bash script.

8. Commit the changes and push to the github repo.

```bash
git add --all
git commit -m 'update to sample_prep'
git push
```
9. Repeat the steps the same as in the "test run" example above, except that you work on the new project repo.









