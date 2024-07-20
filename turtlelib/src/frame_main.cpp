#include <fstream>
#include <iostream>
#include <sstream>
#include "turtlelib/svg.hpp"
using namespace std;
using namespace turtlelib;

int main(void){
    // Creating an a frame to give a frame of reference
    Transform2D Ta;

    // Asking the user to enter Tab
    cout << "Please enter T_{a_b}:" << endl;

    // Taking that input into a string and making the transform
    string TabString;
    getline(cin, TabString);
    stringstream TabStringStream(TabString);

    Transform2D Tab;
    
    TabStringStream >> Tab;

    // Asking the user to enter Tbc
    cout << "Please enter T_{b_c}:" << endl;

    // Taking that input into a string and making the transform
    string TbcString;
    getline(cin, TbcString);
    stringstream TbcStringStream(TbcString);

    Transform2D Tbc;
    
    TbcStringStream >> Tbc;

    Transform2D Tac = Tab * Tbc;

    // Outputting info about the frames
    cout << "T_{a_b}: " << Tab << endl;
    cout << "T_{b_a}: " << Tab.inv() << endl;
    cout << "T_{b_c}: " << Tbc << endl;
    cout << "T_{c_b}: " << Tbc.inv() << endl;
    cout << "T_{a_c}: " << Tac << endl;
    cout << "T_{c_a}: " << Tac.inv() << endl;

    // 

    // Asking for point p_a
    cout << "Please enter p_a:" << endl;
    Point2D pa;

    // Taking that input into a string and making the point
    string paString;
    getline(cin, paString);
    stringstream paStringStream(paString);
    
    paStringStream >> pa;

    // Making the points in frame b and c
    Point2D pb = Tab.inv()(pa);
    Point2D pc = Tac.inv()(pa);

    // Outputting info about point pa
    cout << "p_a: " << pa << endl;
    cout << "p_b: " << pb << endl;
    cout << "p_c: " << pc << endl;

    // Vector

    // Asking for vecctor v_b
    cout << "Please enter v_b:" << endl;
    Vector2D vb;

    // Taking that input into a string and making the vector
    string vbString;
    getline(cin, vbString);
    stringstream vbStringStream(vbString);
    
    vbStringStream >> vb;

    // Outputting the normalized vector
    Vector2D v_bhat = normalize(vb);

    cout << "v_bhat: " << v_bhat << endl;

    // Making the points in frame b and c
    Vector2D va = Tab(vb);
    Vector2D vc = Tbc.inv()(vb);

    // Outputting info about twist twa
    cout << "v_a: " << va << endl;
    cout << "v_b: " << vb << endl;
    cout << "v_c: " << vc << endl;

    // Twists

    // Asking for twist tw_b
    cout << "Please enter tw_b:" << endl;
    Twist2D twb;

    // Taking that input into a string and making the twist
    string twbString;
    getline(cin, twbString);
    stringstream twbStringStream(twbString);
    
    twbStringStream >> twb;

    // Making the points in frame b and c
    Twist2D twa = Tab(twb);
    Twist2D twc = Tbc.inv()(twb);

    // Outputting info about point va
    cout << "V_a: " << twa << endl;
    cout << "V_b: " << twb << endl;
    cout << "V_c: " << twc << endl;

    // Drawing everything
    Svg mySvg;

    mySvg(Ta, "a");
    mySvg(Tab, "a_b");
    mySvg(Tac, "a_c");

    mySvg(pa, "purple");
    mySvg(pb, "brown");
    mySvg(pc, "orange");

    mySvg(vb, Ta, "purple");
    mySvg(vb, Tab, "brown");
    mySvg(vb, Tac, "orange");

    mySvg.writeToFile("frame_main");

    return 1;
}