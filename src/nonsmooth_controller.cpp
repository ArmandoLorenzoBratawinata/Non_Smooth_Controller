#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <control_toolbox/pid.h>
#include <linearizing_controllers_msgs/DynamicsLinearizingControllerStatus.h>

static Eigen::Vector3d x_odom;  //variavel que salva valores da pose da odometria
//static double x_odom[3];

//static Eigen::Vector3d velocidade;
static double velocidade [3];

//static double x_ref[3] = {2, 2, 0}; //referencia forcada da pose
static Eigen::Vector3d x_ref; //referencia forcada da pose



//função callback (CB) para subscriber da pose da odometria
void odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    
    //    double aux1, aux2;
/*
    //variaveis auxliares para pegar a velocidade u linear
    aux1 = odom_msg->twist.twist.linear.x; //publicado como u[0]*cos(x[2]);
    aux2 = odom_msg->twist.twist.linear.y; //publicado como u[0]*sin(x[2]);

    velocidade [0] = sqrt(aux1*aux1+aux2*aux2); //operação para eliminar os senos, cossenos e ficar somente com u
    velocidade[1] = odom_msg->twist.twist.angular.z;
*/

    //pegando a pose estimada, x, y, theta
    x_odom[0] = odom_msg->pose.pose.position.x;
    x_odom[1] = odom_msg->pose.pose.position.y;
    x_odom[2] = odom_msg->pose.pose.orientation.z;
}

//função callback (CB) para subscriber para as velocidades
void velocidadeCB(const linearizing_controllers_msgs::DynamicsLinearizingControllerStatus::ConstPtr &vel)
{
    // pegando velocidade linear em x
    velocidade[0] = vel->process_value.linear.x;
    ROS_WARN ("%f", velocidade[0]);

    //pegando velocidade angular em z
    velocidade[1] = vel->process_value.angular.z;
    ROS_ERROR ("%f", velocidade[1]);
}

void refCB(const geometry_msgs::Pose2D::ConstPtr &ref)
{
    x_ref << ref->x, ref->y, ref->theta;
}


int main (int argc, char **argv)
{

    ros::init(argc,argv,"nonsmooth"); //nome do nodo
    ros::NodeHandle node;       //cria o nodo

    //subscriber para odometria, tópico:/dynam.../odom, mensagem: nav_msgs::odometry
    ros::Subscriber sub_pose = node.subscribe("/dynamics_linearizing_controller/odom",1,&odomCB);
    ros::Subscriber sub_velocidade = node.subscribe("/dynamics_linearizing_controller/status",1,&velocidadeCB);
    ros::Subscriber sub_pose_ref = node.subscribe("/ref",1,&refCB);

    //publisher para velocidades no modelo linearizado, escreve no tópico /dynamics_li..../command
    ros::Publisher pub = node.advertise<geometry_msgs::Accel>("/dynamics_linearizing_controller/command",1);

    ros::Publisher pub_sinais_controle = node.advertise <geometry_msgs::Pose2D>("/controle",1);
    ros::Publisher pub_sinais_erro = node.advertise <geometry_msgs::Pose2D>("/erro",1);
    geometry_msgs::Pose2D controle, erro;

    //formula de ganhos em funcao de Ts, sem overshoot
    //Kp = 2*4.6/Ts
    //Ki = (4.6/Ts)^2
    //Ts escolhido: 50 ms

    double Kp[2] = {184, 184};
    //double Kp[2] = {18, 18};
    double Ki[2] = {8464, 8464};
    //double Ki[2] = {84, 84};

    control_toolbox::Pid pid_linear;
    control_toolbox::Pid pid_angular;

    pid_linear.initPid(Kp[0], Ki[0], 0, 1000000, -1000000); //coloca-se os ganhos e os limites superior e inferior do integrador
    pid_angular.initPid(Kp[1], Ki[1], 0, 1000000, -1000000);


    geometry_msgs::Accel v; //acelerações calculadas resultantes do controle

    double gamma[2] = {0.25, 1};   //valores escolhidos para a parte do controle nao linear
    double lambda[2] = {0.25, 1};

    x_ref <<0,0,0;
    //x_ref << 0.1, 0.1, 0.2; //referencia forcada da pose

    ros::Rate loop(100);
    ros::Time time_anterior = ros::Time::now();

    while(ros::ok())
    {
        //mudanca de coordenadas
        Eigen::Matrix3d R;      //matriz de rotacao
        R << cos(x_ref[2]), sin(x_ref[2]), 0,
                -sin(x_ref[2]), cos(x_ref[2]), 0,
                0, 0, 1;

        //faz a mudanca

        Eigen::Vector3d x_barra = R*(x_odom-x_ref);

        //muda pra polar
        double e_polar = sqrt(x_barra[0]*x_barra[0] + x_barra[1]*x_barra[1]);
        double psi = atan2(x_barra[1], x_barra[0]);
        double alpha = x_barra[2] - psi;


        //parte nao linear do controle
        double u[2] = {(-gamma[0]*e_polar*cos(alpha)),
                       (-gamma[1]*alpha - gamma[0]*cos(alpha)*sin(alpha)+gamma[0]*(lambda[1]/lambda[0])*cos(alpha)*(sin(alpha)/alpha)*psi)};

        //u[0]=0.0; //testes para o PI
        //u[1]=0.0;

        ros::Time time = ros::Time::now();//tempo para PI

        //double e[2] = {(ref[0] - velocidade[0]), (ref[1] - velocidade[1])}; //calcula erro

        double e[2] = {(u[0] - velocidade[0]), (u[1] - velocidade[1])}; //calcula o erro
//        if (e[0]<0.05 || e[1]< 0.05){e[0]=0; e[1]=0;} //estabiliza pid


        v.linear.x = pid_linear.computeCommand(e[0],time - time_anterior);   //aplica PI
        v.angular.z = pid_angular.computeCommand(e[1], time - time_anterior);

        time_anterior = time;       //atualiza tempo

        pub.publish(v);     //publica


        //publica sinais de controle e erro para gerar os gráficos
  //debug pid
        /*erro.x=e[0];
        erro.theta=e[1];
        controle.x=v.linear.x;
        controle.theta=v.angular.z;
*/

        erro.x=x_barra[0];
        erro.y=x_barra[1];
        erro.theta=x_barra[2];

        pub_sinais_erro.publish(erro);
        //pub_sinais_controle.publish(controle);

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
