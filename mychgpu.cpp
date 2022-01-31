// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen, Ruochun Zhang
// =============================================================================
// Chrono::Gpu demo using SMC method. A body whose geometry is described by an
// OBJ file is time-integrated in Chrono and interacts with a granular wave tank
// in Chrono::Gpu via the co-simulation framework. The entire simulation consists
// of 2 runs: the settling phase (which outputs a checkpoint file), and a restarted
// phase (which load the checkpoint file and then drop the ball, literally).
// =============================================================================

#include <iostream>
#include <vector>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

//me
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChCoordsys.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/motion_functions/ChFunction_Sine.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearDriveline.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrAssetConverter.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


using namespace chrono;
using namespace chrono::gpu;
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;


// Output frequency
float out_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 2000;







collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;




////////////////////////////////////////////////////////////////////////////////////////////////TwoAugurs
void runBallDrop(ChSystemGpuMesh& gpu_sys, ChGpuSimulationParameters& params) {
    // Create a ChronoENGINE physical system
    ChSystemSMC mphysicalSystem;
    mphysicalSystem.SetCollisionSystemType(collision_type);

    // Contact material shared among all objects
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    //RotScale: our case usually 1
    // dimentions from solidworks are like 30 mm but chrono understands it as 30 cm
    float statorrotscale =0.1;
    float augur1rotscale = 0.1;
    float augur2rotscale = 0.1;

    //radius for volume calculation:
    //2.5
    float stator_radius =0.25;
    float augur1_radius = 0.75;
    float augur2_radius = 0.75;

    //height for volume calculation:
    float stator_height = 0.25;
    float augur1_height =5;
    float augur2_height = 5;
   
    
    // Density:
    float stator_density = 1 * params.sphere_density;
    float augur1_density = 1 * params.sphere_density;
    float augur2_density = 1 * params.sphere_density;

    //MASS
    float stator_mass = (float)(CH_C_PI * stator_radius * stator_radius * stator_height * stator_density)*1;
    float augur1_mass = 1*(float)(CH_C_PI * augur1_radius * augur1_radius * augur1_height * augur1_density) * 1;
    float augur2_mass = 1*(float)(CH_C_PI * augur2_radius * augur2_radius * augur2_height * augur2_density) * 1;
    //inertia
    double stator_inertia = 1.0 / 12.0 * augur1_mass * ((3 * augur1_radius * augur1_radius) + augur1_height * augur1_height);
    double augur1_inertiay = 1.0 / 2.0 * (augur1_mass * augur1_radius * augur1_radius);
    double augur1_inertiax = 1.0 / 12.0 * augur1_mass * ((3 * augur1_radius * augur1_radius) + augur1_height * augur1_height);
    double augur1_inertiaz = augur1_inertiax;
    double augur2_inertiay = 1.0 / 2.0 * (augur2_mass * augur2_radius * augur2_radius);
    double augur2_inertiax = 1.0 / 12.0 * augur2_mass * ((3 * augur2_radius * augur2_radius) + augur2_height * augur2_height);
    double augur2_inertiaz = augur2_inertiax;

   

   gpu_sys.AddMesh(GetChronoDataFile("models/stator3_2.obj"), ChVector<float>(0), ChMatrix33<float>(statorrotscale),
       stator_mass);
   //augur1_2:augur1_s
   gpu_sys.AddMesh(GetChronoDataFile("models/augur_1_5cm_2.obj"), ChVector<float>(0), ChMatrix33<float>(augur1rotscale),
       augur1_mass);
   //augur2:augur2_s
   gpu_sys.AddMesh(GetChronoDataFile("models/augur_2_5cm_2.obj"), ChVector<float>(0), ChMatrix33<float>(augur2rotscale),
       augur2_density);


   

    // One more thing: we need to manually enable mesh in this run, because we disabled it in the settling phase,
    // let's overload that option.
   gpu_sys.EnableMeshCollision(true);

    gpu_sys.Initialize();
    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;


  
    ChVector<> positionA3(0, 0,0);
    mphysicalSystem.Set_G_acc(ChVector<>(0, 0, -980));
   // Body for Stator1

    std::shared_ptr<ChBody>stator3(mphysicalSystem.NewBody());
    stator3->SetMass(augur1_mass);
    //-35
    stator3->SetPos(ChVector<>(0, 0,0));
    //stator3->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    stator3->SetBodyFixed(false);
    stator3->SetInertiaXX(ChVector<>(stator_inertia, stator_inertia, stator_inertia));
    
    mphysicalSystem.AddBody(stator3);

    // Body for Augur 1, named Rotator 3
    std::shared_ptr<ChBody> rotor3(mphysicalSystem.NewBody());
    rotor3->SetPos(ChVector<>(0, 0.5* (stator_height), 0));
    //-35
    rotor3->SetMass(augur1_mass);
    rotor3->SetBodyFixed(false);
    mphysicalSystem.AddBody(rotor3);
    rotor3->SetInertiaXX(ChVector<>(augur1_inertiax, augur1_inertiay, augur1_inertiaz));

    //// Body for Augur 2, named Rotator 2
    std::shared_ptr<ChBody> rotor2(mphysicalSystem.NewBody());
    //-35
    rotor2->SetPos(ChVector<>(0, -0.5 * (stator_height),0));
    rotor2->SetMass(augur2_mass);
    rotor2->SetBodyFixed(false);
    rotor2->SetInertiaXX(ChVector<>(augur2_inertiax, augur2_inertiay, augur2_inertiaz));
    mphysicalSystem.AddBody(rotor2);



    // Create the motor for augur1
    auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(rotor3,                // body A (slave)
        stator3,               // body B (master)
        ChFrame<>(stator3->GetPos(), Q_from_AngX(CH_C_PI / 2))); // motor frame, in abs. coords
    mphysicalSystem.Add(rotmotor1);
    //rotmotor3->SetSpindleConstraint(true,true, true, true, false);
    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed =
        chrono_types::make_shared<ChFunction_Const>(-1*CH_C_PI);  // constant angular speed, in [rad/s], 1PI/s =180°/s
    // Let the motor use this motion function:
    rotmotor1->SetSpeedFunction(mwspeed);


    //// Create the motor for augur2
    auto rotmotor2 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    // Connect the rotor and the stator and add the motor to the system:
    rotmotor2->Initialize(rotor2,                // body A (slave)
        stator3,               // body B (master)
        ChFrame<>(stator3->GetPos(), Q_from_AngX(CH_C_PI / 2))); // motor frame, in abs. coords
    mphysicalSystem.Add(rotmotor2);
    //rotmotor3->SetSpindleConstraint(true,true, true, true, false);
    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed2 =
        chrono_types::make_shared<ChFunction_Const>(1*CH_C_PI);  // constant angular speed, in [rad/s], 1PI/s =180°/s
    // Let the motor use this motion function:
    rotmotor2->SetSpeedFunction(mwspeed2);



    rotor2->SetCollide(true);
    rotor3->SetCollide(true);


     //Create rigid ball_body simulation
    //ChSystemSMC sys_cylinder;
    mphysicalSystem.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    //mphysicalSystem.Set_G_acc(ChVector<>(0, 0, 0));






     



 







    ChGpuVisualization gpu_vis(&gpu_sys, &mphysicalSystem);
    if (render) {
        gpu_vis.SetTitle("Chrono::Gpu ball cosim demo");
        gpu_vis.SetCameraPosition(ChVector<>(0, -200, 100), ChVector<>(0, 0, 0));
        gpu_vis.SetCameraMoveScale(1.0f);
        gpu_vis.Initialize();
    }

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    float iteration_step = params.step_size;
    std::cout << "Output at    " << out_fps << " FPS" << std::endl;
    std::cout << "Rendering at " << render_fps << " FPS" << std::endl;
    unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);

    int currframe = 0;
    int currframee = 0;
    unsigned int curr_step = 0;



    clock_t start = std::clock();

    /////////////me
    //gpu_sys.EnableMeshCollision(true);
    gpu_sys.SetRecordingContactInfo(true);



    ////meshforceINFO:
    //    /// Set sphere-to-mesh static friction coefficient.
    //gpu_sys.SetStaticFrictionCoeff_SPH2MESH(2.0);
    ///// Set sphere-to-mesh rolling friction coefficient.
    //gpu_sys.SetRollingCoeff_SPH2MESH(2.0);
    ///// Set sphere-to-mesh spinning friction coefficient.
    //gpu_sys.SetSpinningCoeff_SPH2MESH(2.0);

    ///// Set sphere-to-mesh normal contact stiffness.
    //gpu_sys.SetKn_SPH2MESH(2.2);
    ///// Set sphere-to-mesh normal damping coefficient.
    //gpu_sys.SetGn_SPH2MESH(2.0);

    ///// Set sphere-to-mesh tangential contact stiffness.
    //gpu_sys.SetKt_SPH2MESH(2.0);
    ///// Set sphere-to-mesh tangential damping coefficient.
    //gpu_sys.SetGt_SPH2MESH(2.0);

    ///// Set the ratio of adhesion force to sphere weight for sphere to mesh.
    //gpu_sys.SetAdhesionRatio_SPH2MESH(2.0);

    gpu_sys.SetBDFixed(true);
    int n = 0;
    ChQuaternion<double> rotme(1, 0, 0, 0);
    gpu_sys.GetNumSDs();
    

    //rotor3->SetPos(ChVector <>(0, 0,0));

    //mphysicalSystem.Set_G_acc(ChVector<>(0, 0, -980));
    int i = 0;
    chrono::ChMatrixNM<double, 150, 3> myforceR;
    chrono::ChMatrixNM<double, 150, 1> myforcexR;
    chrono::ChMatrixNM<double, 150, 1> myforceyR;
    chrono::ChMatrixNM<double, 150, 1> myforcezR;
    chrono::ChMatrixNM<double, 150, 1> PositionxR;
    chrono::ChMatrixNM<double, 150, 1> PositionyR;
    chrono::ChMatrixNM<double, 150, 1> PositionzR;
    chrono::ChMatrixNM<double, 150, 3> mytourqeR;
    chrono::ChMatrixNM<double, 150, 3> mytourqexR;
    chrono::ChMatrixNM<double, 150, 3> mytourqeyR;
    chrono::ChMatrixNM<double, 150, 3> mytourqezR;

    chrono::ChMatrixNM<double, 150, 3> myforceL;
    chrono::ChMatrixNM<double, 150, 1> myforcexL;
    chrono::ChMatrixNM<double, 150, 1> myforceyL;
    chrono::ChMatrixNM<double, 150, 1> myforcezL;
    chrono::ChMatrixNM<double, 150, 1> PositionxL;
    chrono::ChMatrixNM<double, 150, 1> PositionyL;
    chrono::ChMatrixNM<double, 150, 1> PositionzL;
    chrono::ChMatrixNM<double, 150, 3> mytourqeL;
    chrono::ChMatrixNM<double, 150, 3> mytourqexL;
    chrono::ChMatrixNM<double, 150, 3> mytourqeyL;
    chrono::ChMatrixNM<double, 150, 3> mytourqezL;

    myforceR.setZero();
    myforcexR.setZero();
    myforceyR.setZero();
    myforcezR.setZero();
    myforceL.setZero();
    myforcexL.setZero();
    myforceyL.setZero();
    myforcezL.setZero();
    PositionyR.setZero();
    PositionxR.setZero();
    PositionzR.setZero();
    PositionyL.setZero();
    PositionxL.setZero();
    PositionzL.setZero();
    mytourqeR.setZero();
    mytourqexR.setZero();
    mytourqeyR.setZero();
    mytourqezR.setZero();
    mytourqeL.setZero();
    mytourqexL.setZero();
    mytourqeyL.setZero();
    mytourqezL.setZero();

 
    for (double t = 0; t <= params.time_end; t += iteration_step, curr_step++) {

        stator3->SetRot(ChMatrix33 <>(0, ChVector < >(0, 0, 1)));
        //stator3->SetPos_dt(ChVector<>(0, 0, -50));
        //rotor3->SetPos_dt(ChVector<>(0, 0, -50));
        //rotor2->SetPos(ChVector<>(rotor3->GetPos()[0], rotor2->GetPos()[1], rotor3->GetPos()[2]));
      
       // gpu_sys.ApplyMeshMotion(3, stator4->GetPos(), stator4->GetRot(), stator4->GetPos_dt(),
            //stator4->GetWvel_par());
        gpu_sys.ApplyMeshMotion(0, stator3->GetPos(), stator3->GetRot(), stator3->GetPos_dt(),
            stator3->GetWvel_par());
        gpu_sys.ApplyMeshMotion(1, rotor3->GetPos(), rotor3->GetRot(), rotor3->GetPos_dt(),
            rotor3->GetWvel_par());
       gpu_sys.ApplyMeshMotion(2, rotor2->GetPos(), rotor2->GetRot(), rotor2->GetPos_dt(),
            rotor2->GetWvel_par());
       

        //rotor3->SetPos(ChVector <> (0,0,1000-35-n*2));

        // the above function ses the current velocity and position, so you need to set it const each loop. Otherwise, it wil
        //follow the previuos step
        // // no rotation
        //cylinder_body->SetRot(ChQuaternion <>(1,0,0,0) );
        //stator3->SetRot(ChMatrix33 <>(0, ChVector < >(0, 0, 1)));
        //stator3->SetPos(ChVector <> (0,0,-t*2));
        //guide1->SetPos_dt(ChVector<>(0, 0, -2));
        //n++;


        ChVector<>  stator3_force;
        ChVector<>  stator3_torque;
        gpu_sys.CollectMeshContactForces(0, stator3_force, stator3_torque);
        stator3->Empty_forces_accumulators();
        stator3->Accumulate_force(stator3_force, stator3->GetPos(), false);
        stator3->Accumulate_torque(stator3_torque, false);

        ChVector<>  rotor3_force;
        ChVector<>   rotor3_torque;
        gpu_sys.CollectMeshContactForces(1, rotor3_force, rotor3_torque);
        std::cout << rotor3_force[1] << endl;
        rotor3->Empty_forces_accumulators();
        rotor3->Accumulate_force(rotor3_force, rotor3->GetPos(), false);
        rotor3->Accumulate_torque(rotor3_torque, false);

        ChVector<>  rotor2_force;
        ChVector<>  rotor2_torque;
        gpu_sys.CollectMeshContactForces(2, rotor2_force, rotor2_torque);
        rotor2->Empty_forces_accumulators();
        rotor2->Accumulate_force(rotor2_force, rotor2->GetPos(), false);
        rotor2->Accumulate_torque(rotor2_torque, false);



        //std::cout << rotor3_force[1] << endl;


      

        if (curr_step % out_steps == 0) {
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            char mesh_filename[100];
            sprintf(filename, "%s/step180N%06d.csv", out_dir.c_str(), currframe);
            sprintf(mesh_filename, "%s/step180N%06d_mesh", out_dir.c_str(), currframe++);
            gpu_sys.WriteParticleFile(std::string(filename));
            gpu_sys.WriteMeshes(std::string(mesh_filename));
            std::cout << "Force: The Right Augur" << endl;
            std::cout << rotor3_force[0] << endl;
            std::cout << rotor3_force[1] << endl;
            std::cout << rotor3_force[2] << endl;
            std::cout << "Force: Stator" << endl;
            std::cout << stator3_force[0] << endl;
            std::cout << stator3_force[1] << endl;
            std::cout << stator3_force[2] << endl;
            std::cout << myforceR << endl;
            //one matrix
            myforceR((i * 3)) = rotor3_force[0];
            myforceR((i * 3) + 1) = rotor3_force[1];
            myforceR((i * 3) + 2) = rotor3_force[2];

            mytourqeR((i * 3)) = rotor3_torque[0];
            mytourqeR((i * 3) + 1) = rotor3_torque[1];
            mytourqeR((i * 3) + 2) = rotor3_torque[2];
        
            myforcexR(i) = rotor3_force[0];
            myforceyR(i) = rotor3_force[1];
            myforcezR(i) = rotor3_force[2];

            mytourqexR(i) = rotor3_torque[0];
            mytourqeyR(i) = rotor3_torque[1];
            mytourqezR(i) = rotor3_torque[2];



            std::cout << "Force:The Left Augur" << endl;
            std::cout << rotor2_force[0] << endl;
            std::cout << rotor2_force[1] << endl;
            std::cout << rotor2_force[2] << endl;
            std::cout << myforceL << endl;
            //one matrix
            myforceL((i * 3)) = rotor2_force[0];
            myforceL((i * 3) + 1) = rotor2_force[1];
            myforceL((i * 3) + 2) = rotor2_force[2];


            mytourqeL((i * 3)) = rotor2_torque[0];
            mytourqeL((i * 3) + 1) = rotor2_torque[1];
            mytourqeL((i * 3) + 2) = rotor2_torque[2];


            myforcexL(i) = rotor2_force[0];
            myforceyL(i) = rotor2_force[1];
            myforcezL(i) = rotor2_force[2];


            mytourqexL(i) = rotor2_torque[0];
            mytourqeyL(i) = rotor2_torque[1];
            mytourqezL(i) = rotor2_torque[2];

            std::cout << "Position y:The Right Augur" << endl;
            PositionyR(i) = rotor3->GetPos()[1];
            PositionxR(i) = rotor3->GetPos()[0];
            PositionzR(i) = rotor3->GetPos()[2];
            std::cout << PositionyR << endl;
            std::cout << "Position y:The Left Augur" << endl;
            PositionyL(i) = rotor2->GetPos()[1];
            PositionxL(i) = rotor2->GetPos()[0];
            PositionzL(i) = rotor2->GetPos()[2];
            std::cout << PositionyL << endl;







            std::cout << rotor3->GetRot() << endl;

            i++;

            char filename2[100];
            sprintf(filename2, "%s/contact%06d.csv", out_dir.c_str(), currframee++);
            gpu_sys.WriteContactInfoFile(std::string(filename2));

            n++;
            
        }
        

        if (render && curr_step % render_steps == 0) {
            if (gpu_vis.Render())

                break;
        }



        
        gpu_sys.AdvanceSimulation(iteration_step);
        mphysicalSystem.DoStepDynamics(iteration_step);
    }



/////////////////////output to file:
#include <iostream>
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib> // for exit function
// This program output values from an array to a file named example2.dat


ofstream outdata; // outdata is like cin
int j; // loop index
//int num[5] = { 4, 3, 6, 7, 12 }; // list of output values

outdata.open("ForceRightAugur180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforceR(j) << endl;
outdata.close();

outdata.open("ForceRightAugurX180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforcexR(j) << endl;
outdata.close();

outdata.open("ForceRightAugurY180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforceyR(j) << endl;
outdata.close();

outdata.open("FroceRightAugurZ180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforcezR(j) << endl;
outdata.close();

outdata.open("TourqeRightAugurX180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << mytourqexR(j) << endl;
outdata.close();
outdata.open("TouqreRightAugurY180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << mytourqeyR(j) << endl;
outdata.close();
outdata.open("TouqreRightAugurZ180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << mytourqezR(j) << endl;

outdata.close();
outdata.open("TouqreRightAugur180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << mytourqeR(j) << endl;


outdata.close();
outdata.open("ForceLeftAugur180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforceL(j) << endl;
outdata.close();

outdata.open("ForceLeftAugurX180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforcexL(j) << endl;
outdata.close();

outdata.open("ForceLeftAugurY180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforceyL(j) << endl;
outdata.close();

outdata.open("ForceLeftAugurZ180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << myforcezL(j) << endl;
outdata.close();


outdata.open("TourqeLeftAugurX720.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << mytourqexL(j) << endl;
outdata.close();
outdata.open("TouqreLeftAugurY180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << mytourqeyL(j) << endl;
outdata.close();
outdata.open("TouqreLeftAugurZ180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << mytourqezL(j) << endl;
outdata.close();

///////////////////position
outdata.open("Right Augur Positionx180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << PositionxR(j) << endl;
outdata.close();
outdata.open("Right Augur Positiony180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << PositionyR(j) << endl;
outdata.close();

outdata.open("Right Augur Positionz180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j <151; ++j)
    outdata << PositionzR(j) << endl;
outdata.close();

outdata.open("Left Augur Positionx180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j <151; ++j)
    outdata << PositionxL(j) << endl;
outdata.close();

outdata.open("Left Augur Positiony180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j < 151; ++j)
    outdata << PositionyL(j) << endl;
outdata.close();

outdata.open("Left Augur Positionz180N.dat"); // opens the file
if (!outdata) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
}

for (j = 0; j <151; ++j)
    outdata << PositionzL(j) << endl;
outdata.close();
















//////////////////////////////////////////////////////////




    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;
}

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;
    if (argc != 2 || ParseJSON(gpu::GetDataFile(argv[1]), params) == false) {
        std::cout << "Usage:\n./demo_GPU_ballcosim <json_file>" << std::endl;
        return 1;
    }

    if (params.run_mode > 1) {
        printf("ERROR! Unknown run_mode specified!\n");
        return 2;
    }

    // Output directory
    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    std::string checkpoint_file = out_dir + "/checkpoint.dat";

    //if (params.run_mode == 1) {
        // run_mode = 1, this is a restarted run

        // Load checkpoint file.
        // Note that with current version, user defined meshes and boundaries are not stored in the checkpoint file,
        // so they must be manually set later. This behavior will be improved in later patches.
        // Simulation parameters and particle states are all in with this file loaded.
    //uncoooooomennnnnnnnnnt
        //ChSystemGpuMesh gpu_sys(checkpoint_file);

        // Add a ball through a mesh, whose dynamics are managed by Chrono Core, and run this co-simulation.
       
        //now exits main funtion!!:
       
        
        //return 0;
    //}

    // run_mode = 0, this is a newly started run. We have to set all simulation params.
    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            ChVector<float>(params.box_X, params.box_Y, params.box_Z));

    printf(
        "Now run_mode == 0, this run is particle settling phase.\n"
        "After it is done, you will have a settled bed of granular material.\n"
        "A checkpoint file will be generated in the output directory to store this state.\n"
        "You can then open the JSON file, change \"run_mode\" from 0 to 1, then run this demo again,\n"
        "to proceed with the ball drop part of this demo.\n\n");
    gpu_sys.SetRecordingContactInfo(true);
    float iteration_step = params.step_size;

    // z box is the from bottom to cylinder and fill top is where at the top particles begin till to fill bottom
    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = params.box_Z / 4.0;

    chrono::utils::PDSampler<float> sampler(2.4f * params.sphere_radius);
    // chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);

    // leave a 4cm margin at edges of sampling
    ChVector<> hdims(params.box_X / 2 - 4.0, params.box_Y / 2 - 4.0, 0);
    ChVector<> center(0, 0, fill_bottom + 2.0 * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    // Shift up for bottom of box
    center.z() += 3 * params.sphere_radius;
    while (center.z() < -0.85) {
        // You can uncomment this line to see a report on particle creation process.
         std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    center.z() = 0.95;
    while (center.z() < fill_top) {
        // You can uncomment this line to see a report on particle creation process.
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }
    std::cout << body_points.size() << " particles sampled!" << std::endl;

    gpu_sys.SetParticles(body_points);

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);

    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);

    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));

    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    // gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    // gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    // gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    gpu_sys.SetParticleOutputMode(params.write_mode);
    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.SetBDFixed(true);

    // In the settling run we disable the mesh.
    gpu_sys.EnableMeshCollision(true);


    //meee
    gpu_sys.SetRecordingContactInfo(true);
    /*
    // We could prescribe the motion of the big box domain. But here in this demo we will not do that.
    std::function<double3(float)> pos_func_wave = [&params](float t) {
        double3 pos = {0, 0, 0};

        double t0 = 0.5;
        double freq = CH_C_PI / 4;

        if (t > t0) {
            pos.x = 0.1 * params.box_X * std::sin((t - t0) * freq);
        }
        return pos;
    };

    gpu_sys.setBDWallsMotionFunction(pos_func_wave);
    */

    gpu_sys.Initialize();

    unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);
    int currframe = 0;
    int currframee = 0;
    unsigned int curr_step = 0;


    runBallDrop(gpu_sys, params);


  /*  for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        if (curr_step % out_steps == 0) {
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            
            sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe++);
            gpu_sys.WriteParticleFile(std::string(filename));


 
        }


        gpu_sys.AdvanceSimulation(iteration_step);
    }*/


   // out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    //total_frames = (unsigned int)(params.time_end * out_fps);
    //currframee = 0;
    //curr_step = 0;

    //for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        //if (curr_step % out_steps == 0) {
           // char filename2[100];
            //sprintf(filename2, "%s/contact%06d.csv", out_dir.c_str(), currframee++);
            //gpu_sys.WriteContactInfoFile(std::string(filename2));
        //}
   // }

    // This is settling phase, so output a checkpoint file
    gpu_sys.WriteCheckpointFile(checkpoint_file);

    return 0;
}
