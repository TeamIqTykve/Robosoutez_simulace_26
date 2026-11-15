using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

public interface IRobot
{
    void MoveForward(double time);
    void MoveBackward(double time);
    void MoveUntillWall();
    void RotateHand(int where);
    void Turn(double angle);
    void MoveUntillLine(int count,/*int color,*/ bool forward);
    void TurnAndRotate(double angle, int where);
    bool IsExecuting();
    bool IsAlive();
}

