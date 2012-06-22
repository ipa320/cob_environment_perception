#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Core>

#define PI 3.14159265
using namespace std;

float
toDegrees (float radians)
{
  return radians * (180 / PI);
}

float
toRadians (float degrees)
{
  return degrees * (PI / 180);
}
struct euler
{
  double x;
  double y;
  double z;
};
struct quart
{
  double q1;
  double q2;
  double q3;
  double q4;
};
using namespace std;

int
main ()
{
  Eigen::Vector3f v1 (-0.33, 0.33, -0.34);
  cout << "vector v1 : \n" << v1 << endl;
  //cout<< "    "<<v1[0]<<"  "<<"  "<<v1[1]<<"  "<<v1[2]<<endl;
  //Eigen::Vector3f v1_unit(0,0,0);
  /*
   float v1_mag, v1_unit_mag;
   v1_mag = v1.norm();
   cout<<" v1_mag : "<<v1_mag<<endl;
   v1_unit = v1/v1_mag;
   */
  v1.normalize ();
  //v1_unit_mag = v1_unit.norm();

  cout << "normalized vector v1 :\n" << v1 << endl;
  //cout<<"v1_unit: "<<v1_unit<<endl;
  float cos_x, cos_y, cos_z;
  cos_x = acos (v1[0]);
  cos_y = acos (v1[1]);
  cos_z = acos (v1[2]);
  cout << " cos_x: " << toDegrees (cos_x) << " cos_y: " << toDegrees (cos_y) << " cos_z: " << toDegrees (cos_z) << endl;

  euler a;
  quart b;
  a.x = toRadians(50);
  a.y = toRadians(90);
  a.z = toRadians(0);

  cout << endl << "Euler1:" << endl;
  cout << a.x << endl << a.y << endl << a.z << endl;

  cout << "Quart:" << endl;
  //b.q1 = -cos ((a.x - a.y) / 2) * sin (a.z / 2);
  b.q1 = cos (a.x / 2) * cos (a.y / 2)* cos (a.z / 2) + sin (a.x / 2) * sin (a.y / 2)* sin (a.z / 2);
  //cout << "q1: " << b.q1 << endl;

  //b.q2 = -sin ((a.x - a.y) / 2) * sin (a.z / 2);
  b.q2 = sin (a.x / 2) * cos (a.y / 2)* cos (a.z / 2) - cos (a.x / 2) * sin (a.y / 2)* sin (a.z / 2);
  //cout << "q2: " << b.q2 << endl;

  //b.q3 = -sin ((a.x + a.y) / 2) * cos (a.z / 2);
  b.q3 = cos (a.x / 2) * sin (a.y / 2)* cos (a.z / 2) + sin (a.x / 2) * cos (a.y / 2)* sin (a.z / 2);
  //cout << "q3: " << b.q3 << endl;

  //b.q4 = cos ((a.x + a.y) / 2) * cos (a.z / 2);
  b.q4 = cos (a.x / 2) * cos (a.y / 2)* sin (a.z / 2) - sin (a.x / 2) * sin (a.y / 2)* cos (a.z / 2);
  //cout << "q4: " << b.q4 << endl;

  cout << "q1 : " << b.q1 << endl;
  cout << "q2 : " << b.q2 << endl;
  cout << "q3 : " << b.q3 << endl;
  cout << "q4 : " << b.q4 << endl;


  a.x = atan2 ((b.q1 * b.q3 + b.q2 * b.q4), (b.q2 * b.q3 - b.q1 * b.q4));
  a.y = acos (pow (-b.q1, 2) + pow (-b.q2, 2) + pow (-b.q3, 2) + pow (-b.q4, 2));
  a.z = -atan2 ((b.q1 * b.q3 - b.q2 * b.q4), (b.q2 * b.q3 + b.q1 * b.q4));
  /*
  a.x = atan (2*(b.q1 * b.q2 + b.q3 * b.q4)/(1-(2*(b.q2*b.q2) + (b.q3*b.q3))));
  a.y = asin (2*(b.q1 * b.q3 - b.q4 * b.q2));
  a.z = atan (2*(b.q1 * b.q4 + b.q2 * b.q3)/(1-(2*(b.q3*b.q3) + (b.q4*b.q4))));
*/

  cout << endl << "Euler2:" << endl;
  cout << toDegrees (a.x) << endl << toDegrees (a.y) << endl << toDegrees (a.z) << endl;

  return 0;
}
