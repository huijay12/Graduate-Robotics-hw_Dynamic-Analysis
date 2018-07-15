//============================================================================
// Name        : Robotics_dynamic.cpp
// Author      : lee
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <armadillo>
#include <cmath>
#include <fstream>

using namespace std;
using namespace arma;


double pi = 3.1416;
double d4 = 50;
double a1 = 350;
double a2 = 350;

double g = 9.8;

double q134(double w4, double w5){

	return (-1)*atan(w4/w5);
}

double q3(double w1, double w2){

	return (-1)*acos(((w1*w1+w2*w2)-2*a1*a2)/(2*a1*a2));
}

double q1(double w1, double w2, double q3){

	return atan((w2*(cos(q3)+1)-w1*sin(q3))/(w1*(cos(q3)+1)+w2*sin(q3)));
}

double q4(double q134, double q1, double q3){

	return q134 - q1 - q3;
}

double d2(double w3, double d4){

	return w3 - d4;
}

double dot(double qT, double qTminus1){

	return qT - qTminus1;
}


int main() {

	fstream torque1, torque2, torque3, torque4;
	torque1.open("torque1.csv", ios::out);
	torque2.open("torque2.csv", ios::out);
	torque3.open("torque3.csv", ios::out);
	torque4.open("torque4.csv", ios::out);

	double x = -200, y = 350, z = 0;
	double alpha = -1, beta = 0, gama = 0;

	double dx = 0.5, dy = 0, dz = 15/40;
	double dalpha = 0.0025, dbeta = 0.0025, dgama = 0;

//	double q134Ary[420], q3Ary[420], q1Ary[420], q4Ary[420], d2Ary[420];
	double qAry[820][5], q134Ary[820], qDotAry[820][5], qDoubleDotAry[820][5];



    for(int i=0; i<=400; i++){


    	q134Ary[i] = q134(alpha, beta);

    	//q3
    	qAry[i][3] = q3(x, y);

    	//q1
    	qAry[i][1] = q1(x, y, qAry[i][3]);

    	//q4
    	qAry[i][4] = q4(q134Ary[i], qAry[i][1], qAry[i][3]);

    	//q2
        qAry[i][2] = d2(z, d4);

    	x += dx;
    	y += dy;
    	z += dz;
    	//alpha += dalpha;
    	beta += dbeta;

    }



    for(int i=401; i<800; i++){


    	q134Ary[i] = q134(alpha, beta);

    	//q3
    	qAry[i][3] = q3(x, y);

    	//q1
    	qAry[i][1] = q1(x, y, qAry[i][3]);

    	//q4
    	qAry[i][4] = q4(q134Ary[i], qAry[i][1], qAry[i][3]);

    	//q2
        qAry[i][2] = d2(z, d4);

    	x += dx;
    	y += dy;
    	z += dz;
    	alpha += dalpha;
    	//beta -= dbeta;



    }

    cout << "q done" << endl;


    for(int i=1; i<800; i++){

    	qDotAry[i][1] = dot(qAry[i][1], qAry[i-1][1]);
    	qDotAry[i][2] = dot(qAry[i][2], qAry[i-1][2]);
    	qDotAry[i][3] = dot(qAry[i][3], qAry[i-1][3]);
    	qDotAry[i][4] = dot(qAry[i][4], qAry[i-1][4]);

    }
    cout << "qDot done" << endl;

    for(int i=2; i<800; i++){

    	qDoubleDotAry[i][1] = dot(qDotAry[i][1], qDotAry[i-1][1]);
    	qDoubleDotAry[i][2] = dot(qDotAry[i][2], qDotAry[i-1][2]);
    	qDoubleDotAry[i][3] = dot(qDotAry[i][3], qDotAry[i-1][3]);
    	qDoubleDotAry[i][4] = dot(qDotAry[i][4], qDotAry[i-1][4]);

    }
    cout << "qDoubleDot done" << endl;





    /*******STEP 1********/

	int xi[5] = {0, 1, 0, 1, 1};  //1:revolute joint  0:prismatic joint
	double m[5] = {0, 2, 1, 1, 0.5};
	double d[5];

	mat T[5];
	mat T01, T12, T23, T34;
	mat R[5];
	mat H1;
	mat DHat[5];
	mat D[5];

	vec i3 = vec(3);
	vec i4 = vec(4);
	vec vz[5];
	vec vg[5];
	vec deltaS[5];
	vec deltaR[5];
	vec C[5];
	vec mg[5];

	vec v[5];
	vec vDot[5];
	vec w[5];
	vec wDot[5];

	vec f[6];
	vec n[6];
	double torque[6];

	i3(0) = 0;
	i3(1) = 0;
	i3(2) = 1;

	i4(0) = 0;
	i4(1) = 0;
	i4(2) = 0;
	i4(3) = 1;




	for(int i=0; i<5; i++){

		R[i] = mat(3, 3);
		v[i] = vec(3);
		vDot[i] = vec(3);
		w[i] = vec(3);
		wDot[i] = vec(3);
		vz[i] = vec(3);
		vg[i] = vec(3);
		deltaS[i] = vec(3);
		deltaR[i] = vec(3);
		C[i] = vec(3);
		DHat[i] = mat(3, 3);
		D[i] = mat(3, 3);

		vg[i](0) = 0;    vg[i](1) = 0;    vg[i](2) = 0;

	}



	for(int i=0; i<6; i++){

		f[i] = vec(3);
		n[i] = vec(3);
	}

	T01 = mat(4, 4);
	T12 = mat(4, 4);
	T23 = mat(4, 4);
	T34 = mat(4, 4);

	v[0].fill(0);               //v0 = 0
	w[0].fill(0);               //w0 = 0

	vDot[0](0) = 0;
	vDot[0](1) = 0;
	vDot[0](2) = -g;

    f[5](0) = -10*g;             //outer force
    f[5](1) = -10*g;
    f[5](2) = -10*g;

    n[5](0) = -10*g;             //outer moment
    n[5](1) = -10*g;
    n[5](2) = -10*g;

    H1 = mat(3, 4);
	H1(0, 0) = 1;    H1(0, 1) = 0;    H1(0, 2) = 0;    H1(0, 3) = 0;
	H1(1, 0) = 0;    H1(1, 1) = 1;    H1(1, 2) = 0;    H1(1, 3) = 0;
	H1(2, 0) = 0;    H1(2, 1) = 0;    H1(2, 2) = 1;    H1(2, 3) = 0;

	//orientation of mass matrix






    for(int t=2; t<800; t++){

    	T01(0, 0) = cos(qAry[t][1]);    T01(0, 1) = -1*sin(qAry[t][1]);    T01(0, 2) = 0;     T01(0, 3) = 0;
    	T01(1, 0) = sin(qAry[t][1]);    T01(1, 1) = cos(qAry[t][1]);       T01(1, 2) = 0;     T01(1, 3) = 0;
    	T01(2, 0) = 0;                  T01(2, 1) = 0;                     T01(2, 2) = 1;     T01(2, 3) = 0;
    	T01(3, 0) = 0;                  T01(3, 1) = 0;                     T01(3, 2) = 0;     T01(3, 3) = 1;

    	T12(0, 0) = 1;                  T12(0, 1) = 0;                     T12(0, 2) = 0;     T12(0, 3) = a1;
    	T12(1, 0) = 0;                  T12(1, 1) = 1;                     T12(1, 2) = 0;     T12(1, 3) = 0;
    	T12(2, 0) = 0;                  T12(2, 1) = 0;                     T12(2, 2) = 1;     T12(2, 3) = qAry[t][2];
    	T12(3, 0) = 0;                  T12(3, 1) = 0;                     T12(3, 2) = 0;     T12(3, 3) = 1;

    	T23(0, 0) = cos(qAry[t][3]);    T23(0, 1) = -1*sin(qAry[t][3]);    T23(0, 2) = 0;     T23(0, 3) = a2*cos(qAry[t][3]);
    	T23(1, 0) = sin(qAry[t][3]);    T23(1, 1) = cos(qAry[t][3]);       T23(1, 2) = 0;     T23(1, 3) = a2*sin(qAry[t][3]);
    	T23(2, 0) = 0;                  T23(2, 1) = 0;                     T23(2, 2) = 1;     T23(2, 3) = qAry[t][2];
    	T23(3, 0) = 0;                  T23(3, 1) = 0;                     T23(3, 2) = 0;     T23(3, 3) = 1;

    	T34(0, 0) = cos(qAry[t][4]);    T34(0, 1) = 0;                     T34(0, 2) = -1*sin(qAry[t][4]);    T34(0, 3) = 0;
    	T34(1, 0) = sin(qAry[t][4]);    T34(1, 1) = 0;                     T34(1, 2) = cos(qAry[t][4]);       T34(1, 3) = 0;
    	T34(2, 0) = 0;                  T34(2, 1) = -1;                    T34(2, 2) = 0;                     T34(2, 3) = -1*d4;
    	T34(3, 0) = 0;                  T34(3, 1) = 0;                     T34(3, 2) = 0;                     T34(3, 3) = 1;

    	T[0] = eye<mat>(4, 4);    //T00 = I
    	T[1] = T[0]*T01;
    	T[2] = T[1]*T12;
    	T[3] = T[2]*T23;
    	T[4] = T[3]*T34;

    	R[0] = eye<mat>(3, 3);   //R00 = I




    	 /*********STEP 2************/

        for(int k=1; k<=4; k++){


    	    R[k-1](0, 0) = T[k-1](0, 0);    R[k-1](0, 1) = T[k-1](0, 1);    R[k-1](0, 2) = T[k-1](0, 2);
    	    R[k-1](1, 0) = T[k-1](1, 0);    R[k-1](1, 1) = T[k-1](1, 1);    R[k-1](1, 2) = T[k-1](1, 2);
    	    R[k-1](2, 0) = T[k-1](2, 0);    R[k-1](2, 1) = T[k-1](2, 1);    R[k-1](2, 2) = T[k-1](2, 2);

    	    vz[k-1] = R[k-1]*i3;

    	    w[k] = w[k-1] + xi[k]*qDotAry[t][k]*vz[k-1];

    	    wDot[k] = wDot[k-1] + xi[k]*(qDoubleDotAry[t][k] + cross(w[k-1], qDotAry[t][k]*vz[k-1]));

    	    deltaS[k] = H1*(T[k] - T[k-1])*i4;

    	    vDot[k] = vDot[k-1] + cross(wDot[k], deltaS[k]) + cross(w[k], cross(w[k], deltaS[k])) + (1-xi[k])*(qDoubleDotAry[t][k]*vz[k-1] + cross(2*w[k], qDotAry[t][k]*vz[k-1]));


        }
        cout << "step 2 done" << endl;

        /*********STEP 4************/

        //mass center vector of links
    	C[1](0) = 0;
    	C[1](1) = 0;
    	C[1](2) = 0.5 * qAry[t][2];

    	C[2](0) = 0.5 * a1 * cos(qAry[t][1]);
    	C[2](1) = 0.5 * a1 * sin(qAry[t][1]);
    	C[2](2) = qAry[t][2];

    	C[3](0) = a1 * cos(qAry[t][1]) + 0.5*a2*cos(qAry[t][1]+qAry[t][3]);
    	C[3](1) = a1 * sin(qAry[t][1]) + 0.5*a2*sin(qAry[t][1]+qAry[t][3]);
    	C[3](2) = qAry[t][2];

    	C[4](0) = a1 * cos(qAry[t][1]) + a2*cos(qAry[t][1]+qAry[t][3]);
    	C[4](1) = a1 * sin(qAry[t][1]) + a2*sin(qAry[t][1]+qAry[t][3]);
    	C[4](2) = qAry[t][2] - 0.5*d4;

    	//distance from former coordinate
    	d[1] = 0.5 * qAry[t][2];
    	d[2] = 0.5 * a1;
    	d[3] = 0.5 * a2;
    	d[4] = 0.5 * d4;


	    R[4](0, 0) = T[4](0, 0);    R[4](0, 1) = T[4](0, 1);    R[4](0, 2) = T[4](0, 2);
	    R[4](1, 0) = T[4](1, 0);    R[4](1, 1) = T[4](1, 1);    R[4](1, 2) = T[4](1, 2);
	    R[4](2, 0) = T[4](2, 0);    R[4](2, 1) = T[4](2, 1);    R[4](2, 2) = T[4](2, 2);

		DHat[1](0, 0) = 0;    DHat[1](0, 1) = 0;    DHat[1](0, 2) = 0;    //no
		DHat[1](1, 0) = 0;    DHat[1](1, 1) = 0;    DHat[1](1, 2) = 0;
		DHat[1](2, 0) = 0;    DHat[1](2, 1) = 0;    DHat[1](2, 2) = 0;

		DHat[2](0, 0) = 0;    DHat[2](0, 1) = 0;    DHat[2](0, 2) = 0;    //x
		DHat[2](1, 0) = 0;    DHat[2](1, 1) = 1;    DHat[2](1, 2) = 0;
		DHat[2](2, 0) = 0;    DHat[2](2, 1) = 0;    DHat[2](2, 2) = 1;

		DHat[3](0, 0) = 0;    DHat[3](0, 1) = 0;    DHat[3](0, 2) = 0;    //x
		DHat[3](1, 0) = 0;    DHat[3](1, 1) = 1;    DHat[3](1, 2) = 0;
		DHat[3](2, 0) = 0;    DHat[3](2, 1) = 0;    DHat[3](2, 2) = 1;

		DHat[4](0, 0) = 1;    DHat[4](0, 1) = 0;    DHat[4](0, 2) = 0;    //y
		DHat[4](1, 0) = 0;    DHat[4](1, 1) = 0;    DHat[4](1, 2) = 0;
		DHat[4](2, 0) = 0;    DHat[4](2, 1) = 0;    DHat[4](2, 2) = 1;

        for(int k=4; k>=1; k--){

        	deltaR[k] = C[k] - H1*T[k]*i4;
        	cout << "deltaR " << endl << deltaR[k] << endl;

        	vg[k](2) = -1*m[k]*g;
        	f[k] = f[k+1] + m[k]*(vDot[k] + cross(wDot[k], deltaR[k]) + cross(w[k], cross(w[k], deltaR[k]))) -vg[k];
        	cout << "f " << endl << f[k] << endl;

        	cout << m[k]*d[k]*d[k]/12 << endl;

        	DHat[k](0, 0) = m[k]*d[k]*d[k]/12 * DHat[k](0, 0);
        	DHat[k](1, 1) = m[k]*d[k]*d[k]/12 * DHat[k](1, 1);
        	DHat[k](2, 2) = m[k]*d[k]*d[k]/12 * DHat[k](2, 2);
        	cout << "DHat" << endl << DHat[k] << endl;

        	D[k] = R[k]*DHat[k]*R[k].t();
        	cout << "D " << endl << D[k] << endl;

        	n[k] = n[k+1] + cross((deltaS[k]+deltaR[k]), f[k]) - cross(deltaR[k], f[k+1]) + D[k]*wDot[k] + cross(w[k], D[k]*w[k]);
        	cout << "n " << endl << n[k] << endl;

        	torque[k] = as_scalar(xi[k]*n[k].t()*vz[k-1] + (1-xi[k])*f[k].t()*vz[k-1]);
        	cout << "torque " << endl << torque[k] << endl;

        }

        torque1 << torque[1]/1000 << endl;
        torque2 << torque[2]/1000 << endl;
        torque3 << torque[3]/1000 << endl;
        torque4 << torque[4]/1000 << endl;


        cout << "step 4 done" << endl;

    }   //end of for t=2~799


    torque1.close();
    torque2.close();
    torque3.close();
    torque4.close();

	return 0;
}
