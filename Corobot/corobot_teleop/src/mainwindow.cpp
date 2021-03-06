#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ImageBig.h"
#include "Image.h"

#ifdef Q_WS_WIN
#include "windows.h"
#include "mmsystem.h"
#endif


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //set the parent widgets to the different widget created
    r.setParent(this);
    ui->setupUi(this);
    arm.setParent(ui->frame);
    arm_rotation.setParent(ui->frame_6);
    gripper.setParent(ui->frame_7);
    wrist.setParent(ui->frame_8);
    gps.setParent(ui->webView);
    gps.setList(ui->listWidget);
    hokuyo.setParent(ui->frame_hokuyo);
    gpsUrlTImer.invalidate();
    //hokuyo.setGeometry(1000,1000,200,200);

    ui->tabWidget_5->setFocus();

    LRF_lines_show = false;
    IR_data_show = false;
    //arm.show();
    //gripper.show();irCallback
    //wrist.show();

    //connect Q_SIGNALS to Q_SLOTS.
    connect(&arm,SIGNAL(theta1(double)),ui->lcdNumber,SLOT(display(double))); //display shoulder angle
    connect(&arm,SIGNAL(theta2(double)),ui->lcdNumber_2,SLOT(display(double))); //display elbow angle
    connect(&wrist,SIGNAL(angle(double)),ui->lcdNumber_3,SLOT(display(double))); //display wrist angle
    connect(ui->radioButton_5,SIGNAL(toggled(bool)),&arm,SLOT(shoulder_degree(bool))); //choose to display shoulder angle in degree or radian
    connect(ui->radioButton_7,SIGNAL(toggled(bool)),&arm,SLOT(elbow_degree(bool)));//choose to display elbow angle in degree or radian
    connect(ui->radioButton_3,SIGNAL(toggled(bool)),&wrist,SLOT(degree(bool))); //choose to display the wrist angle in degree or radian
    //connect(ui->verticalSlider,SIGNAL(valueChanged(int)),ui->lcdNumber_6,SLOT(display(int))); // display the forward speed percent value
    //connect(ui->horizontalSlider,SIGNAL(valueChanged(int)),ui->lcdNumber_7,SLOT(display(int)));//display the angular speed percent value
    connect(ui->spinBox_2,SIGNAL(valueChanged(int)),(QObject*)&gps,SLOT(set_zoom(int))); //set zoom value for the gps map
    connect(&gps,SIGNAL(url_changed(QUrl)),this,SLOT(change_url(QUrl))); //set the rl to the web viewer
    connect(ui->comboBox,SIGNAL(currentIndexChanged(QString)),&gps,SLOT(set_map_type(QString)));//set the gps map type
    connect(ui->radioButton,SIGNAL(toggled(bool)),&r,SLOT(moveGripper(bool)));//open or close the gripper
    connect(ui->radioButton_9,SIGNAL(toggled(bool)),&arm,SLOT(Corobot(bool)));//gives robot information(Corobot or Explorer) to the arm widget
    connect(ui->radioButton_9,SIGNAL(toggled(bool)),&armCircle,SLOT(Corobot(bool)));//gives robot information(Corobot or Explorer) to the arm circle widget
    //connect(ui->radioButton_9,SIGNAL(toggled(bool)),ui->checkBox,SLOT(setEnabled(bool)));//disable arm control for kinect control if Explorer
    connect(ui->radioButton_9,SIGNAL(toggled(bool)),&r,SLOT(corobot(bool)));//gives robot information(Corobot or Explorer) to the ros widget

    //connect(ui->checkBox_2,SIGNAL(toggled(bool)),this,SLOT(show_kinect_tabs(bool))); //hide kinect tabs
    //connect(ui->checkBox_2,SIGNAL(toggled(bool)),&r,SLOT(select_kinect(bool))); //gives info to ros widget about kinect selection

    connect(&r,SIGNAL(gps_lat(double)),ui->lcdNumber_4,SLOT(display(double))); //display the latitude
    connect(&r,SIGNAL(gps_lon(double)),ui->lcdNumber_5,SLOT(display(double)));//display the longitude
    connect(&r,SIGNAL(gps_coord(double,double)),&gps,SLOT(update_coord(double,double))); //gives new coordinate to the gps widget
    connect(&r,SIGNAL(griperState(int)),&gripper,SLOT(setState(int))); // get gripper state from the robot and display it
    connect(&wrist,SIGNAL(angle_rad(float)),&r,SLOT(turnWrist(float))); //tirn wrist if the wrist widget angle changed


    connect(ui->connect,SIGNAL(clicked()),this,SLOT(connect_clicked()));//connect button pushed
    connect(&arm,SIGNAL(posarm(float,float)),&armCircle,SLOT(setpos(float,float))); //same position of the arm for the two widgets


    connect(this,SIGNAL(size_changed()),&gps,SLOT(check_size()));//adjust the size of the map with the size of the window

    connect(ui->save_gps,SIGNAL(clicked()),&gps,SLOT(save_clicked())); //save button clicked
    connect(ui->start_gps,SIGNAL(clicked()),&gps,SLOT(start_clicked()));//start button clicked
    connect(ui->sto_gps,SIGNAL(clicked()),&gps,SLOT(stop_clicked()));//stop button clicked
    connect(ui->load_gps,SIGNAL(clicked()),&gps,SLOT(load_clicked())); //load button clicked

    connect(ui->listWidget,SIGNAL(itemSelectionChanged()),&gps,SLOT(selection_changed()));//selection of the itinerary changed

    connect(&r,SIGNAL(battery_percent(int)),ui->progressBar,SLOT(setValue(int))); //display the battery percent
    connect(&r,SIGNAL(battery_volts(double)),ui->lcdNumber_8,SLOT(display(double)));//display the battery voltage

    connect(&r,SIGNAL(update_ptzcam(QImage)),this,SLOT(update_ptz(QImage))); //force to update the ptz cam
    connect(&r,SIGNAL(update_mapimage(QImage)),this,SLOT(update_map(QImage))); //force to update the ptz cam
    connect(&r,SIGNAL(update_rearcam(QImage)),this,SLOT(update_rear(QImage))); //force to update the reat cam scene
    connect(&r,SIGNAL(update_kinectRGBcam(QImage)),this,SLOT(update_kinectRGB(QImage))); //force to update the kinect RGB scene
    connect(&r,SIGNAL(update_kinectDepthcam(QImage)),this,SLOT(update_kinectDepth(QImage))); //force to update the kinect Depth scene
    connect(ui->camera, SIGNAL(currentChanged(int)),&r, SLOT(currentCameraTabChanged(int))); //Subscribe only to the camera topic we are interested in

    //***************************************************************************
    // Arm control
    connect(&arm,SIGNAL(shoulderAngle_rad(double)),&r,SLOT(moveShoulderArm(double)));
    connect(&arm_rotation,SIGNAL(armAngle_rad(double)),&r,SLOT(rotateArm(double)));
    connect(&arm,SIGNAL(elbowAngle_rad(double)),&r,SLOT(moveElbowArm(double)));
    //connect(ui->ArmResetButton,SIGNAL(clicked()),&arm,SLOT(arm_reset()));
    connect(ui->ArmResetButton,SIGNAL(clicked()),&r,SLOT(ResetArm()));
    connect(&r,SIGNAL(arm_model(bool,bool,bool,bool)),&arm,SLOT(setModel(bool,bool,bool,bool)));

    //***************************************************************************
        //MISC tab
    //***************************************************************************
    //Bumper information
    connect(&r,SIGNAL(bumper_update(int,int,int,int)),this,SLOT(bumper_update_slot(int,int,int,int)));

    //IR data
    connect(&r,SIGNAL(irData(double,double)),this,SLOT(irdata_update_slot(double,double)));
    connect(&r,SIGNAL(irData(double,double)),&hokuyo,SLOT(IR_update(double,double)));

    //Spatial data
    connect(&r,SIGNAL(spatial_data(double,double,double,double,double,double,double,double,double)),this,SLOT(spatial_update_slot(double,double,double,double,double,double,double,double,double)));


    //***************************************************************************


    //****************************************************************************
    //Pan/Tilt contorl

    connect(ui->Pan_control_bar,SIGNAL(valueChanged(int)),this,SLOT(Pan_control(int)));
    connect(ui->Tilt_control_bar,SIGNAL(valueChanged(int)),this,SLOT(Tilt_control(int)));
    connect(ui->PanTilt_Reset,SIGNAL(clicked()),this,SLOT(Pan_Tilt_reset()));

    //****************************************************************************


    //***************************************************************************
    //Main motor contorl

    connect(ui->Forward, SIGNAL(pressed()), &r, SLOT(increase_speed()));
    connect(ui->Forward, SIGNAL(released()), &r, SLOT(decrease_speed()));
    connect(ui->BACKWARD, SIGNAL(pressed()), &r, SLOT(increase_backward_speed()));
    connect(ui->BACKWARD, SIGNAL(released()), &r, SLOT(decrease_speed()));

    connect(ui->TURNLEFT,SIGNAL(pressed()),&r,SLOT(turn_left()));
    connect(ui->TURNRIGHT,SIGNAL(pressed()),&r,SLOT(turn_right()));
    connect(ui->TURNLEFT,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->TURNRIGHT,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->STOP,SIGNAL(clicked()),&r,SLOT(motor_stop()));

    connect(ui->speed_fast, SIGNAL(toggled(bool)), &r, SLOT(setSpeedFast(bool)));
    connect(ui->speed_moderate, SIGNAL(toggled(bool)), &r, SLOT(setSpeedModerate(bool)));
    connect(ui->speed_slow, SIGNAL(toggled(bool)), &r, SLOT(setSpeedSlow(bool)));

    connect(ui->Forward_2, SIGNAL(pressed()), &r, SLOT(increase_speed()));
    connect(ui->Forward_2, SIGNAL(released()), &r, SLOT(decrease_speed()));
    connect(ui->BACKWARD_2, SIGNAL(pressed()), &r, SLOT(increase_backward_speed()));
    connect(ui->BACKWARD_2, SIGNAL(released()), &r, SLOT(decrease_speed()));

    connect(ui->TURNLEFT_2,SIGNAL(pressed()),&r,SLOT(turn_left()));
    connect(ui->TURNRIGHT_2,SIGNAL(pressed()),&r,SLOT(turn_right()));
    connect(ui->TURNLEFT_2,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->TURNRIGHT_2,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->STOP_2,SIGNAL(clicked()),&r,SLOT(motor_stop()));

    connect(ui->speed_fast_2, SIGNAL(toggled(bool)), &r, SLOT(setSpeedFast(bool)));
    connect(ui->speed_moderate_2, SIGNAL(toggled(bool)), &r, SLOT(setSpeedModerate(bool)));
    connect(ui->speed_slow_2, SIGNAL(toggled(bool)), &r, SLOT(setSpeedSlow(bool)));

    //****************************************************************************


    //*******************************************************************************
    //Hokuyo update Tab
    connect(&r,SIGNAL(hokuyo_update(Hokuyo_Points*)),&hokuyo,SLOT(hokuyo_update(Hokuyo_Points*)));

    //buttons configuration
    connect(ui->ShowLRFlines,SIGNAL(clicked()),this,SLOT(showLRFlines()));
    connect(ui->ShowIRdata,SIGNAL(stateChanged(int)),this,SLOT(showIRdata(int)));
    connect(ui->showLRF,SIGNAL(stateChanged(int)),this,SLOT(showLRFdata(int)));

    //*******************************************************************************

    connect(&r,SIGNAL(velocity_info(double,double)),this,SLOT(encoder_info_update(double,double)));

    connect(ui->quitButton,SIGNAL(clicked()),this,SLOT(close()));

    //*******************************************************************************
    //Motor Control Toggle
    connect(ui->MotorControlToggle,SIGNAL(stateChanged(int)),this,SLOT(motor_control_toggle(int)));

    //***********************************************************************************


    //create different scenes
    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 0, 640, 480);

    QGraphicsScene *scene3 = new QGraphicsScene(this);
    scene3->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene3->setSceneRect(0, 00, 640, 480);

    QGraphicsScene *scene4 = new QGraphicsScene(this);
    scene4->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene4->setSceneRect(0, 00, 640, 480);

    QGraphicsScene *scene10 = new QGraphicsScene(this);
    scene10->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene10->setSceneRect(0, 00, 640, 480);

    QGraphicsScene *scene2 = new QGraphicsScene(this);
    scene2->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene2->setSceneRect(0, 00, 2048, 2048);

    QGraphicsScene *scene5 = new QGraphicsScene(this);
    scene5->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene5->setSceneRect(0, 00, 640, 480);


    //set scenes to the graphicsView widget
    //ui->graphicsView_6->setScene(armCircle.scene());
    //ui->graphicsView_13->setScene(scene2);
    ui->graphicsView->setScene(scene);
    //ui->graphicsView_14->setScene(scene);
    ui->graphicsView_2->setScene(scene4);
    //ui->graphicModesView_15->setScene(scene4);
    ui->graphicsView_3->setScene(scene3);
    //ui->graphicsView_16->setScene(scene3);
    ui->graphicsView_10->setScene(scene10);
    ui->mapView->setScene(scene2);
    ui->frontView->setScene(scene5);




//ui->graphicsView->show();
//ui->graphicsView_4->show();

next_gripper_state = true;

//start the joystick thread
//j.start();
//j.setArmWidget(&arm);
//j.setRos(&r);
//j.setWristWidget(&wrist);

//change focus policy to always be able to move robot with the keyboard
ui->camera->setFocusPolicy(Qt::NoFocus);
//ui->tabWidget->setFocusPolicy(Qt::NoFocus);
this->setFocusPolicy(Qt::StrongFocus);

//gives scene information to the ros thread
//r.add_allcam_scene(ui->graphicsView_13->scene());
//r.add_camera_info_scene(ui->graphicsView->scene());//for HD cam
r.add_rear_cam_scene(ui->graphicsView_10->scene());//add scene info to REAR cam
r.add_ptz_cam_scene(ui->graphicsView->scene()); // add scene info to PTZ cam
r.add_kinect_rgb_scene(ui->graphicsView_2->scene());
r.add_kinect_depth_scene(ui->graphicsView_3->scene());
r.add_map_image_scene(ui->mapView->scene());
r.add_front_image_scene(ui->frontView->scene());

//r.setRangeWidgetParent(ui->graphicsView_5->viewport());
//r.setRangeWidget2Parent(ui->graphicsView_7->viewport());

ui->tabWidget_5->setFocus();

//this->hokuyo.show();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::connect_clicked(){ // executed when the connect button is pushed
    ui->connect->setText("Disconnect");
    ui->connect->setEnabled(false);
    if ( ui->environmentcheckbox->isChecked() ) {
            r.init();
    } else {
            r.init(ui->Master->text().toStdString(),ui->Host->text().toStdString());
            ui->Master->setReadOnly(true);
            ui->Host->setReadOnly(true);
    }
}

void MainWindow::resizeEvent(QResizeEvent *){//execute this function when the window size changes
    Q_EMIT size_changed();
}

void MainWindow::change_url(QUrl url)//set url to the web viewer
{
	if(gpsUrlTImer.isValid() == false || gpsUrlTImer.elapsed()>3000) //We don't want to do more than one querry every 3s
	{
		gpsUrlTImer.restart();
		gpsUrlTImer.start();
                //ui->webView->setUrl(url); Removed because we don't want to break the Google's terms and conditions
	}
}

void MainWindow::update_ptz(QImage image) //order the ptz camera scene to update
{
    //ui->graphicsView_13->scene()->update(0,0,640,480);
        ((Image*)(ui->graphicsView->scene()->items().at(0)))->setImage(image);
    ui->graphicsView->scene()->update(0,0,ui->graphicsView->scene()->width(),ui->graphicsView->scene()->height());

 //    QImage icopy = image.copy(0,0,image.width(),image.height());
    ((Image*)(ui->frontView->scene()->items().at(0)))->setImage(image);
    ui->frontView->scene()->update(0,0,ui->frontView->scene()->width(),ui->frontView->scene()->height());
}

void MainWindow::update_map(QImage image) //order the map image scene to update
{
    ((Image*)(ui->mapView->scene()->items().at(0)))->setImage(image);
    ui->mapView->scene()->update(0,0,ui->mapView->scene()->width(),ui->mapView->scene()->height());
}

void MainWindow::update_rear(QImage image) //order the rear camera scene to update
{
    //ui->graphicsView_13->scene()->update(0,0,640,480);
        ((Image*)(ui->graphicsView_10->scene()->items().at(0)))->setImage(image);
    ui->graphicsView_10->scene()->update(0,0,ui->graphicsView_10->scene()->width(),ui->graphicsView_10->scene()->height());
}

void MainWindow::update_kinectRGB(QImage image) //order the kinect RGB camera scene to update
{
    //ui->graphicsView_13->scene()->update(0,0,640,480);
        ((Image*)(ui->graphicsView_2->scene()->items().at(0)))->setImage(image);
    ui->graphicsView_2->scene()->update(0,0,ui->graphicsView_2->scene()->width(),ui->graphicsView_2->scene()->height());
}

void MainWindow::update_kinectDepth(QImage image) //order the kinect Depth camera scene to update
{
    //ui->graphicsView_13->scene()->update(0,0,640,480);
        ((Image*)(ui->graphicsView_3->scene()->items().at(0)))->setImage(image);
    ui->graphicsView_3->scene()->update(0,0,ui->graphicsView_3->scene()->width(),ui->graphicsView_3->scene()->height());
}


//#ifdef Q_WS_WIN
//bool MainWindow::winEvent(MSG * message, long * result ) // event windows used to receive joystick messages, only compiled for the windows version
//{

//    unsigned int msg = message->message;
//    WPARAM wp = message->wParam;
//    LPARAM lp = message->lParam;

//    float xPos = LOWORD(lp);
//    float yPos = HIWORD(lp);

//    switch ( msg ){
//        case MM_JOY1MOVE :                     // changed position
//                 if(xpos >0.5)
//                     arm.moveArmLeft();
//                 else if (xpos <-0.5)
//                     arm.moveArmRight();
//                 else if (y > 0.5)
//                     arm.moveArmUp();
//                 else if(y, -0.5)
//                     arm.moveArmDown();
//            break;
//        case MM_JOY1BUTTONDOWN :               // button is down
//            r.openGripper();
//            break;
//        case MM_JOY1BUTTONUP :                 // button is up
//            r.closeGripper();
//            break;

//        case MM_JOY2MOVE :                     // changed position
//            if(xpos >0.5)
//                rui->horizontalSlider->setValue(ui->horizontalSlider->value()+5);
//            else if (xpos <-0.5)
//                ui->horizontalSlider->setValue(ui->horizontalSlider->value()-5);
//            else if (y > 0.5)
//                ui->verticalSlider->setValue(ui->verticalSlider->value()+5);
//            else if(y, -0.5)
//                ui->verticalSlider->setValue(ui->verticalSlider->value()-5);
//            break;
//        case MM_JOY2BUTTONDOimage_viewWN :               // button is down


//            break;
//        case MM_JOY2BUTTONUP :                 // button is up
//        break;
//        default:
//            return false;
//        }
//    return true;


//}
//#endif

void MainWindow::keyReleaseEvent(QKeyEvent *event){//executed each time a keyboard key is release

    switch (event->key()) {
        case Qt::Key_W:
            if(!event->isAutoRepeat())
            {
            	r.decrease_speed();
            }
            break;
        case Qt::Key_S:
            if(!event->isAutoRepeat())
            {
            	r.decrease_speed();
            }
            break;
        case Qt::Key_D:
            if(!event->isAutoRepeat())
            {
            	r.stop_turn();
	    }
            break;
        case Qt::Key_A:
            if(!event->isAutoRepeat())
            {
            	r.stop_turn();
	    }
            break;
        default:
            QWidget::keyReleaseEvent(event);
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event){//executed each time a keyboard key is pushed
    switch (event->key()) {
    case Qt::Key_W:
        if(!event->isAutoRepeat())
        {
		r.increase_speed();
        }
        break;
    case Qt::Key_S:
        if(!event->isAutoRepeat())
        {
		r.increase_backward_speed();
        }
        break;
    case Qt::Key_D:
        if(!event->isAutoRepeat())
        {
        	r.turn_right();
        }
        break;
    case Qt::Key_A:
        if(!event->isAutoRepeat())
        {
        	r.turn_left();
        }
        break;
    case Qt::Key_Left:
	if(r.pan >= -65) 
        	ui->Pan_control_bar->setValue(r.pan - 5);
	else if (r.pan > -70)
		ui->Pan_control_bar->setValue(-70);
        break;
    case Qt::Key_Right:
	if(r.pan <= 65) 
        	ui->Pan_control_bar->setValue(r.pan + 5);
	else if (r.pan < 70)
		ui->Pan_control_bar->setValue(70);
        break;
    case Qt::Key_Up:
	if(r.tilt <= 25) 
        	ui->Tilt_control_bar->setValue(r.tilt + 5);
	else if (r.tilt < 30)
		ui->Tilt_control_bar->setValue(30);
        break;
    case Qt::Key_Down:
	if(r.tilt >= -25) 
        	ui->Tilt_control_bar->setValue(r.tilt - 5);
	else if (r.tilt > -30)
		ui->Tilt_control_bar->setValue(-30);
        break;
    case Qt::Key_J:
        arm.moveArmLeft();
        break;
    case Qt::Key_L:
        arm.moveArmRight();
        break;
    case Qt::Key_I:
        arm.moveArmUp();
        break;
    case Qt::Key_K:
        arm.moveArmDown();
        break;
    case Qt::Key_Space:
        r.motor_stop();
        if(next_gripper_state)
            r.closeGripper();
        else
            r.openGripper();
        next_gripper_state = !next_gripper_state;
        break;
    default:
        QWidget::keyPressEvent(event);
    }
}


void MainWindow::bumper_update_slot(int bumper1, int bumper2, int bumper3, int bumper4)
{
    if(bumper1 != 0) ui->label_19->setText("Bumper 1 HIT!"); else ui->label_19->setText("Bumper 1 SAFE");
    if(bumper2 != 0) ui->label_24->setText("Bumper 2 HIT!"); else ui->label_24->setText("Bumper 2 SAFE");
    if(bumper3 != 0) ui->label_26->setText("Bumper 3 HIT!"); else ui->label_26->setText("Bumper 3 SAFE");
    if(bumper4 != 0) ui->label_40->setText("Bumper 4 HIT!"); else ui->label_40->setText("Bumper 4 SAFE");

}

void MainWindow::Pan_control(int value)
{
    r.pan = value;
    corobot_teleop::PanTilt msg;
    msg.pan = r.pan;
    msg.tilt = r.tilt;
    msg.reset = 0;
    r.pan_tilt_control.publish(msg);
}

void MainWindow::Tilt_control(int value)
{
    r.tilt = value;
    corobot_teleop::PanTilt msg;
    msg.pan = r.pan;
    msg.tilt = r.tilt;
    msg.reset = 0;
    r.pan_tilt_control.publish(msg);
}

void MainWindow::Pan_Tilt_reset()
{
    r.pan = 0;
    r.tilt = 0;

    ui-> Pan_control_bar->setValue(0);
    ui-> Tilt_control_bar ->setValue(0);

    corobot_teleop::PanTilt msg;
    msg.pan = r.pan;
    msg.tilt = r.tilt;
    msg.reset = 1;
    r.pan_tilt_control.publish(msg);
}

void MainWindow::encoder_info_update(double linearVelocity,double angularVelocity)
{
    ui->linear_velocity->display(linearVelocity);
    ui->angular_velocity->display(angularVelocity);
}


// main slot to give values to the two motors
void MainWindow::move_speed_select(int x)
{
    //value need to be determined
    switch(x){
    case 1:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.move_speed_level = 1;
        //this->msgBox.setText("move speed 1");
        //this->msgBox.exec();
        break;
    case 2:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.move_speed_level = 2;
        //this->msgBox.setText("move speed 2");
        //this->msgBox.exec();
        break;
    case 3:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.move_speed_level = 3;
        break;
    case 4:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.move_speed_level = 4;
        break;
    default:
        r.move_speed_level = 0;
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
    }
}

void MainWindow::turning_speed_select(int x)
{
    //value need to be determined
    switch(x){
    case 1:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.turning_speed_level = 1;
        //this->msgBox.setText("turn speed 1");
        //this->msgBox.exec();
        break;
    case 2:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.turning_speed_level = 2;
        //this->msgBox.setText("turn speed 2");
        //this->msgBox.exec();
        break;
    case 3:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.turning_speed_level = 3;
        break;
    case 4:
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
        r.turning_speed_level = 4;
        break;
    default:
        r.turning_speed_level = 0;
        r.left_motor_value = 1000;
        r.right_motor_value = 1000;
    }

}

void MainWindow::motor_control_toggle(int value)
{
    if(value != 0)
    {
        ui->Forward->setEnabled(false);
        ui->BACKWARD->setEnabled(false);
        ui->TURNLEFT->setEnabled(false);
        ui->TURNRIGHT->setEnabled(false);
        ui->STOP->setEnabled(false);
        ui->frame->setEnabled(false);
        ui->frame_7->setEnabled(false);
        ui->frame_8->setEnabled(false);
    }

    else
    {
        ui->Forward->setEnabled(true);
        ui->BACKWARD->setEnabled(true);
        ui->TURNLEFT->setEnabled(true);
        ui->TURNRIGHT->setEnabled(true);
        ui->STOP->setEnabled(true);
        ui->frame->setEnabled(true);
        ui->frame_7->setEnabled(true);
        ui->frame_8->setEnabled(true);
    }

}

void MainWindow::irdata_update_slot(double ir01, double ir02)
{
    ui->lcdNumber_IR01->display(ir01);
    ui->lcdNumber_IR02->display(ir02);
}

void MainWindow::spatial_update_slot(double acc_x, double acc_y, double acc_z, double ang_x, double ang_y, double ang_z, double mag_x, double mag_y, double mag_z)
{
    ui->lcdNumber_acc_x->display(acc_x);
    ui->lcdNumber_acc_y->display(acc_y);
    ui->lcdNumber_acc_z->display(acc_z);
    ui->lcdNumber_ang_x->display(ang_x);
    ui->lcdNumber_ang_y->display(ang_y);
    ui->lcdNumber_any_z->display(ang_z);
    ui->lcdNumber_mag_x->display(mag_x);
    ui->lcdNumber_mag_y->display(mag_y);
    ui->lcdNumber_mag_z->display(mag_z);
}

void MainWindow::showLRFlines()
{
    QString string1 = "No Lines";
    QString string2 = "Show Lines";

    if(LRF_lines_show == false)
    {
        ui->ShowLRFlines->setText(string1);
        LRF_lines_show = true;
        hokuyo.showlines = true;
    }
    else
    {
        LRF_lines_show = false;
        ui->ShowLRFlines->setText(string2);
        hokuyo.showlines = false;
    }

}

void MainWindow::showIRdata(int value)
{

    if(value != 0)
    {
        hokuyo.showIR = true;
        hokuyo.showLRF = false;
        ui->showLRF->setEnabled(false);
    }

    else
    {
        hokuyo.showIR = false;
        //hokuyo.showLRF = false;
        ui->showLRF->setEnabled(true);
    }

}

void MainWindow::showLRFdata(int value)
{
    if(value != 0)
    {
        hokuyo.showIR = false;
        hokuyo.showLRF = true;
        ui->ShowIRdata->setEnabled(false);
    }

    else
    {
        //hokuyo.showIR = false;
        hokuyo.showLRF = false;
        ui->ShowIRdata->setEnabled(true);
    }


}
