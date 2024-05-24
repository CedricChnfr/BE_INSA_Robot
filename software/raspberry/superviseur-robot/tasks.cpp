/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>;;.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TSTARTCAMERA 28
#define PRIORITY_TCAMERA 35
#define PRIORITY_TSTOPCAMERA 28
#define PRIORITY_TBATTERY 19
#define PRIORITY_POSITION 50


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 *
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 *
 * 3- Data flow is probably not optimal
 *
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 *
 * 5- Same behavior existe for ComMonitor::Write !
 *
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 *
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex,
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /*      Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_battery, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_state_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
//    if (err = rt_mutex_create(&mutex_position, NULL)) {
//        cerr << "Error position create: " << strerror(-err) << endl << flush;
//        exit(EXIT_FAILURE);
//    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /*      Semaphors creation                                                  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_camStart, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
     if (err = rt_sem_create(&sem_camStop, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    

    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_camera, "th_camera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_start_camera, "th_start_camera", 0, PRIORITY_TSTARTCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_stop_camera, "th_stop_camera", 0, PRIORITY_TSTOPCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
     if (err = rt_task_create(&th_get_arena, "th_get_arena", 0, PRIORITY_POSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
  
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::GetBatteryTask, this)) {
      cerr << "Error task start: " << strerror(-err) << endl << flush;
       exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_camera, (void(*)(void*)) & Tasks::ScreenCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_start_camera, (void(*)(void*)) & Tasks::StartCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_stop_camera, (void(*)(void*)) & Tasks::StopCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_get_arena, (void(*)(void*)) & Tasks::GetArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
       exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
   
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
   
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
   
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        // Update the battery status
        else if(msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
           rt_mutex_acquire(&mutex_battery, TM_INFINITE); // Acquire the mutex for battery status
           battery = msgRcv->GetID(); // Update the battery status
           rt_mutex_release(&mutex_battery); // Release the mutex for battery status
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_sem_v(&sem_camStart);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_sem_v(&sem_camStop);
        }
         else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_sem_v(&sem_arena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            rt_sem_v(&sem_camStart);
            cout << " Camera Replay" << endl;
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            rt_sem_v(&sem_camStart);
            cout << " Camera Replay with no detection" << endl;
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
   
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        //cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
           
            cout << " move: " << cpMove;
           
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * @brief Get the robot battery
 */
void Tasks::GetBatteryTask(void *arg) {
    int rstart;
   
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush; // Print start message
    rt_task_set_periodic(NULL, TM_NOW, 100000000); // Set task periodicity
    rt_sem_p(&sem_barrier, TM_INFINITE); // Wait for synchronization barrier

    while (1) {
        rt_task_wait_period(NULL); // Wait for task period
        //cout << "Periodic battery updated"; // Print periodic update message
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE); // Acquire robotStarted mutex
        rstart = robotStarted; // Read robotStarted value
        rt_mutex_release(&mutex_robotStarted); // Release robotStarted mutex
        if(rstart==1){ // If robotStarted is 1
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); // Acquire robot mutex
            Message* msg = robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET)); // Create and write battery message to robot
            rt_mutex_release(&mutex_robot); // Release robot mutex
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE); // Acquire monitor mutex
            monitor.Write(msg); // Write message to monitor
            rt_mutex_release(&mutex_monitor); // Release monitor mutex
        }
    }
    cout << endl << flush; // Print end of line
}

void Tasks::StartCamera(void *arg) {
    int rstart;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while(1) {
        rt_sem_p(&sem_camStart, TM_INFINITE);
       
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rstart = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
       
        if(rstart == 1) {
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera = new Camera(sm, 10);

            if(camera->Open()) {
                rt_mutex_release(&mutex_camera);
               
                rt_mutex_acquire(&mutex_state_camera, TM_INFINITE);
                cameraStatus = 1;
                rt_mutex_release(&mutex_state_camera);
               
            }
            else {
                rt_mutex_release(&mutex_camera);
            }
           
        }
    }
}


void Tasks::ScreenCamera(void *arg) {
    int cstate;
    Arena arenaTemp =Arena();
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);

    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
       
        rt_mutex_acquire(&mutex_state_camera, TM_INFINITE);
        cstate = cameraStatus;
        rt_mutex_release(&mutex_state_camera);
        if (cstate == 1) {
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            if ((camera != NULL) && camera->IsOpen()) { 
                Img *img = new Img(camera->Grab());
                rt_mutex_release(&mutex_camera);
                MessageImg *msgImage = new MessageImg(MESSAGE_CAM_IMAGE, img);
                WriteInQueue(&q_messageToMon, msgImage);    
            }
            else {
                rt_mutex_release(&mutex_camera);
            }
        }
    }
}


void Tasks::StopCamera(void *arg) {
    int cstate;
   
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    while(1) {        
        rt_sem_p(&sem_camStop, TM_INFINITE);
       
        rt_mutex_acquire(&mutex_state_camera, TM_INFINITE);
        cstate = cameraStatus;
        cameraStatus = 0;
        rt_mutex_release(&mutex_state_camera);
       
        if(cstate == 1) {
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera->Close();
            delete camera;
            camera = NULL;
           
            rt_mutex_release(&mutex_camera);
        }
    }
}


/**
 * @brief Get the robot battery
 */
void Tasks::GetArenaTask(void *arg) {
    //Position position_robot;
    int CameraOpen;
    Arena arenaTemp ;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush; // Print start message
    rt_sem_p(&sem_barrier, TM_INFINITE); // Wait for synchronization barrier

    while (1) {
        rt_sem_p(&sem_arena, TM_INFINITE);
        cout << " Arena start search" << endl;
            rt_mutex_acquire(&mutex_camera, TM_INFINITE); 
            Img *ImagewithArene = new Img(camera->Grab());  
            rt_mutex_release(&mutex_camera);
            // Rechercher l'arène dans l'image
            arenaTemp  = ImagewithArene->SearchArena();
            ImagewithArene->DrawArena(arenaTemp); 
            MessageImg *msgImageArena = new MessageImg(MESSAGE_CAM_IMAGE, ImagewithArene);
            WriteInQueue(&q_messageToMon, msgImageArena); 
            rt_sem_v(&sem_camStop);
            cout << " Arena updated" << endl;         
    }
    cout << endl << flush; // Print end of line
}


/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

