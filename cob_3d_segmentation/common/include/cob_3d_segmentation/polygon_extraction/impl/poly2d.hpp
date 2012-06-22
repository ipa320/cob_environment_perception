
namespace Contour2D {

  struct spline2D {
    int v, x,y, bf;
  };

  spline2D g_Splines[9][256];

  int SplineMap[]={
                   4, 5,  6,
                   3, -1, 7,
                   2, 1,  0

                   /*2, 1,  0,
                 3, -1, 7,
                 4, 5,  6*/
  };

  bool generateSpline2D() {
    int mapX[]={2,1,0, 0,0, 1, 2,2};
    int mapY[]={2,2,2, 1, 0,0,0, 1};
    //int mapY[]={0,0,0, 1, 2,2,2, 1};

    for(int i=0; i<256; i++) {
      bool b[8];
      for(int j=0; j<8; j++)
        b[j]=i&(1<<j);
      spline2D s={};
      if(!i) {
        g_Splines[8][i]=s;
        continue;
      }
      int a=-1;
      for(int j=3; j>=0; j--) {
        if(b[7-j]) {
          a=7-j;
          break;}
      }
      for(int j=0; a==-1&&j<4; j++) {
        if(b[j]) {
          a=j;
          break;}
      }
      for(int j=3; j>=0; j--) {
        if(b[(a+4+j+8)%8]) {
          s.v=0;
          s.x=mapX[(a+4+j+8)%8]-1;
          s.y=mapY[(a+4+j+8)%8]-1;
          s.bf=(a+j+8)%8;
          a=-1;
          break;
        }
      }
      for(int j=0; a!=-1&&j<4; j++) {
        if(b[(a+4-j+8)%8]) {
          s.v=0;
          s.x=mapX[(a+4-j+8)%8]-1;
          s.y=mapY[(a+4-j+8)%8]-1;
          s.bf=(a-j+8)%8;
          break;
        }
      }
      g_Splines[8][i]=s;
    }

    for(int a=0; a<8; a++) {

      for(int i=0; i<256; i++) {
        bool b[8];
        for(int j=0; j<8; j++)
          b[j]=i&(1<<j);
        spline2D s={};
        if(!i) {
          g_Splines[a][i]=s;
          continue;
        }
        bool found=false;
        for(int j=3; j>=0; j--) {
          if(b[(a+4+j+8)%8]) {
            s.v=-j;
            s.x=mapX[(a+4+j+8)%8]-1;
            s.y=mapY[(a+4+j+8)%8]-1;
            s.bf=(a+j+8)%8;
            found=true;
            break;
          }
        }
        for(int j=0; !found&&j<4; j++) {
          if(b[(a+4-j+8)%8]) {
            s.v=j;
            s.x=mapX[(a+4-j+8)%8]-1;
            s.y=mapY[(a+4-j+8)%8]-1;
            s.bf=(a-j+8)%8;
            break;
          }
        }
        g_Splines[a][i]=s;
      }
    }

    return true;
  }

  static bool bInit = generateSpline2D();

}
