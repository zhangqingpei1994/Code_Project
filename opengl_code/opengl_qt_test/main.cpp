

#include <GL/glut.h>

#include <math.h>
/*const int n = 20;
const GLfloat R = 0.5f;
const GLfloat Pi = 3.1415926536f;*/

/*void myDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT);
        glPolygonMode(GL_FRONT, GL_FILL); // 设置正面为填充模式
        glPolygonMode(GL_BACK, GL_LINE);   // 设置反面为线形模式
        glFrontFace(GL_CCW);               // 设置逆时针方向为正面

        glBegin(GL_POLYGON);               // 按逆时针绘制一个正方形，在左下方
        glVertex2f(-0.5f, -0.5f);
        glVertex2f(0.0f, -0.5f);
        glVertex2f(0.0f, 0.0f);
        glVertex2f(-0.5f, 0.0f);
        glEnd();

        glBegin(GL_POLYGON);               // 按顺时针绘制一个正方形，在右上方
        glVertex2f(0.0f, 0.0f);
        glVertex2f(0.0f, 0.5f);
        glVertex2f(0.5f, 0.5f);
        glVertex2f(0.5f, 0.0f);
        glEnd();

        glFlush();
}*/

#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>


// 太阳、地球和月亮
// 假设每个月都是30天
// 一年12个月，共是360天
static int day = 200; // day的变化：从0到359
void myDisplay(void)
{
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluPerspective(75, 1, 1, 400000000);

    gluLookAt(0, -200000000, 200000000, 0, 0, 0, 0, 0, 1);

    // 绘制红色的“太阳”
    glColor3f(1.0f, 0.0f, 0.0f);
    glutSolidSphere(69600000, 20, 20);
    // 绘制蓝色的“地球”
    glColor3f(0.0f, 0.0f, 1.0f);
    glRotatef(day/360.0*360.0, 0.0f, 0.0f, -1.0f);
    glTranslatef(150000000, 0.0f, 0.0f);
    glutSolidSphere(15945000, 20, 20);
    // 绘制黄色的“月亮”
    glColor3f(1.0f, 1.0f, 0.0f);
    glRotatef(day/30.0*360.0 - day/360.0*360.0, 0.0f, 0.0f, -1.0f);
    glTranslatef(38000000, 0.0f, 0.0f);
    glutSolidSphere(4345000, 20, 20);

    glFlush();

    glutSwapBuffers();
}

void myIdle(void)
{

    ++day;
    if( day >= 360 )
        day = 0;
    myDisplay();
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE); // 修改了参数为GLUT_DOUBLE
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);
    glutCreateWindow("太阳，地球和月亮");    // 改了窗口标题
    glutDisplayFunc(&myDisplay);
    glutIdleFunc(&myIdle);                // 新加入了这句
    glutMainLoop();
    return 0;
}
