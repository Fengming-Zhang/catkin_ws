/* sway */
#include "MotorManagerNode.h"
#include "xform.hpp"

using namespace std;
int MotorManagerNode::generateArmSpeedVal(float speed)
{
    int val =  (int)(speed * 10 * 1111);
    return val;
}

void MotorManagerNode::armNavStep()
{
    ros::Duration delayTime(0.5);
    ros::Duration _tmp_time;
    if (lastArmPath.size() != 0)
    {
        armPlanning = true;
        if (lastArmPath[0].stamp < ros::Time::now())
        {
            ROS_WARN("too slow!!!");

            _tmp_time = ros::Time::now() - lastArmPath[0].stamp;
            delayTime = delayTime + _tmp_time;
            for (int i = 0; i < lastArmPath.size(); i++)
            {
                lastArmPath[i].stamp = lastArmPath[i].stamp + delayTime;
            }
        }
        arm_goto();
    }
    else
    {
        if (armPlanning == true)
        {
            armPlanning = false;
            sz->resetProfile();
            sy->resetProfile();
            el->resetProfile();
            wy->resetProfile();
            wz->resetProfile();
        }
    }

}

void MotorManagerNode::arm_goto()
{
    cout << "before ";
    for (int i = 0; i < 5; i++)
        cout << lastMotorIndex[i] << " ";
    cout << endl;
    for (int i = 0; i < 5; i++)
        motor_goto(i);
    cout << "after ";
    for (int i = 0; i < 5; i++)
        cout << lastMotorIndex[i] << " ";
    cout << endl;
    if (lastMotorIndex[0] *
        lastMotorIndex[1] *
        lastMotorIndex[2] *
        lastMotorIndex[3] *
        lastMotorIndex[4] != 0)
    {
        lastArmPath.erase(lastArmPath.begin());

        for(int i=0; i<5; i++)
            lastMotorIndex[i]--;
    }
}

void MotorManagerNode::motor_goto(int motor_num)
{
    if (lastMotorIndex[motor_num] < lastArmPath.size())
    {
    int index = lastMotorIndex[motor_num];
    int target = lastArmPath[index].arm[motor_num];
    CanMotor* runningMotor;
    switch (motor_num)
    {
        case 0: runningMotor = sz; break;
        case 1: runningMotor = sy; break;
        case 2: runningMotor = el; break;
        case 3: runningMotor = wy; break;
        case 4: runningMotor = wz; break;
        default: exit(0);
    }
    int cur_pose = runningMotor->getPose();
    ros::Duration expect_time;
    expect_time = lastArmPath[index].stamp - ros::Time::now();
    float expect_time_float = expect_time.toSec();
    cout << "time " << expect_time_float << endl;
    if (lastMotorIndex[motor_num] != lastArmPath.size() - 1)
    {
        if (abs(target - cur_pose) < 5)
        {
            cout << abs(target - cur_pose) << endl;
            lastMotorIndex[motor_num]++;
            index++;
            target = lastArmPath[index].arm[motor_num];
            expect_time = lastArmPath[index].stamp - ros::Time::now();
            expect_time_float = expect_time.toSec();
        }
    }
    else if (abs(target - cur_pose) < 2)
    {
        lastMotorIndex[motor_num] = lastArmPath.size();
    }
    int speed = abs(target - cur_pose) / expect_time_float;
    int val = generateArmSpeedVal(speed);
    if (val > 2000000) val = 2000000;
    if (val < 0) val = 0;

    runningMotor->setSpeed(val);
    runningMotor->setupPositionMove(target);
    runningMotor->startPositionMove();
    }
}

void MotorManagerNode::insertArmPoseTarget(int sz, int sy, int el, int wy, int wz, float duration)
{
    ArmTarget _target;
    _target.arm[0] = sz;
    _target.arm[1] = sy;
    _target.arm[2] = el;
    _target.arm[3] = wy;
    _target.arm[4] = wz;
    cout << duration << endl;
    ros::Duration time(duration);
    cout << time.toSec() << endl;
    if (lastArmPath.size() == 0)
        _target.stamp = ros::Time::now() + time;
    else
        _target.stamp = lastArmPath.back().stamp + time;
    lastArmPath.push_back(_target);
}
