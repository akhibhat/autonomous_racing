#include "scan_matching/transform.h"
#include <cmath>
#include "ros/ros.h"
#include <Eigen/Geometry>
#include <complex>
#include <typeinfo>

using namespace std;

void transformPoints(const vector<Point>& points, Transform& t, vector<Point>& transformed_points)
{
    transformed_points.clear();

    for (int i=0; i < points.size(); i++)
    {
        transformed_points.push_back(t.apply(points[i]));
    }
}

int solve_degree2(double a, double b, double c, double& x1, double& x2)
{
    double delta = b*b - 4*a*c;

    if (delta < 0)
    {
        return 0;
    }

    if (delta == 0)
    {
        x1 = -b/(2*a);
        x2 = x1;
        return 1;
    }

    x1 = (-b + sqrt(delta))/(2*a);
    x2 = (-b - sqrt(delta))/(2*a);
    
    return 2;
}

int solve_degree3(double a, double b, double c, double d, double& x0, double& x1, double& x2)
{
    if (a == 0)
    {
        if (b==0)
        {
            if (c ==0)
            {
                return 0;
            }

            x0 = -d/c;
            return 1;
        }

        x2 = 0;
        return solve_degree2(b, c, d, x0, x1);
    }

    // Calculate the normalized form x^3 + a2x^2 + a1x + a0 = 0
    double inv_a = 1. / a;
    double b_a = inv_a * b;
    double b_a2 = b_a * b_a;
    double c_a = inv_a * c;
    double d_a = inv_a * d;

    // Solve the cubic equation
    double Q = (3*c_a - b_a2)/9;
    double R = (9*b_a*c_a - 27*d_a - 2*b_a*b_a2)/54;
    double Q3 = Q*Q*Q;
    double D = Q3 + R*R;
    double b_a_3 = (1. / 3.) * b_a;
    
    if (Q == 0)
    {
        if (R == 0)
        {
            x0 = x1 = x2 = -b_a_3;
            return 3;
        }
        else
        {
            x0 = pow(2 * R, 1/3.0) - b_a_3;
            return 1;
        }
    }

    if (D <= 0)
    {
        double theta = acos(R/sqrt(-Q3));
        double sqrt_Q = sqrt(-Q);
        x0 = 2 * sqrt_Q * cos(theta/3.0) - b_a_3;
        x1 = 2 * sqrt_Q * cos((theta + 2*3.1415)/3.0) - b_a_3;
        x2 = 2 * sqrt_Q * cos((theta + 4*3.1415)/3.0) - b_a_3;

        return 3;
    }

    double AD = pow(fabs(R) + sqrt(D), 1.0/3.0) * (R > 0 ? -1 : (R < 0 ? -1 : 0));
    double BD = (AD == 0) ? 0 : -Q/AD;

    x0 = AD + BD - b_a_3;

    return 1;
}

int solve_degree4(double a, double b, double c, double d, double e,
                    double& x0, double& x1, double& x2, double& x3)
{
    if (a == 0)
    {
        x3 = 0;
        return solve_degree3(b, c, d, e, x0, x1, x2);
    }

    // Normalize coefficients
    double inv_a = 1./a;
    b *= inv_a;
    c *= inv_a;
    d *= inv_a;
    e *= inv_a;

    double b2 = b*b;
    double bc = b*c;
    double b3 = b2*b;

    // Solve resultant cubic
    double r0, r1, r2;
    int n = solve_degree3(1, -c, d*b - 4*e, 4*c*e - d*d - b2*e, r0, r1, r2);
    if (n==0)
    {
        return 0;
    }

    // Calculate R^2
    double R2 = 0.25 * b2 - c + r0;
    double R;

    if (R2 < 0)
    {
        return 0;
    }

    R = sqrt(R2);
    double inv_R = 1./R;

    int nb_real_roots = 0;

    double D2, E2;
    if (R < 10E-12)
    {
        double temp = r0 * r0 - 4*e;
        if (temp < 0)
        {
            D2 = E2 = -1;
        }
        else 
        {
            double sqrt_temp = sqrt(temp);
            D2 = 0.75 * b2 - 2 * c + 2 * sqrt_temp;
            E2 = D2 - 4 * sqrt_temp;
        }
    }
    else 
    {
        double u = 0.75 * b2 - 2 * c - R2,
        v = 0.25 * inv_R * (4 * bc - 8 * d - b3);
        D2 = u + v;
        E2 = u - v;
    }

    double b_4 = 0.25 * b, R_2 = 0.5 * R;
    if (D2 >= 0) 
    {
        double D = sqrt(D2);
        nb_real_roots = 2;
        double D_2 = 0.5 * D;
        x0 = R_2 + D_2 - b_4;
        x1 = x0 - D;
    }

    // Calculate E^2
    if (E2 >= 0) 
    {
        double E = sqrt(E2);
        double E_2 = 0.5 * E;
        if (nb_real_roots == 0) 
        {
        x0 = - R_2 + E_2 - b_4;
        x1 = x0 - E;
        nb_real_roots = 2;
        }
        else 
        {
        x2 = - R_2 + E_2 - b_4;
        x3 = x2 - E;
        nb_real_roots = 4;
        }
    }

    return nb_real_roots;
}

complex<float> get_cubic_root(float a, float b, float c, float d)
{
    float p = c/a - b*b/(3*a*a);
    float q = 2*b*b*b/(27*a*a*a) + d/a - b*c/(3*a*a);

    complex<float> xi(-0.5, sqrt(3)/2);

    complex<float> inside = sqrt(q*q/4 + p*p*p/27);

    complex<float> root;

    for (float k=0; k < 3; ++k)
    {
        root = -b/(3*a) + pow(xi, k) * pow(-q/2.f + inside, 1.f/3.f) + pow(xi, 2.f*k) * pow(-q/2.f - inside, 1.f/3.f);

        if (root.imag() != 0)
        {
            return root;
        }
    }

    return root;
}

float greatest_real_root(float a, float b, float c, float d, float e)
{
    float p = (8*a*c - 3*b*b)/(8*a*a);
    float q = (b*b*b - 4*a*b*c + 8*a*a*d)/(8*a*a*a);
    float r = (-3*b*b*b*b + 256*a*a*a*e - 64*a*a*b*d + 16*a*b*b*c)/(256*a*a*a*a);

    complex<float> m = get_cubic_root(8, 8*p, 2*p*p-8*r, -q*q);

    complex<float> root1 = -b/(4*a) + ( sqrt(2.f*m) + sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
    complex<float> root2 = -b/(4*a) + ( sqrt(2.f*m) - sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
    complex<float> root3 = -b/(4*a) + (-sqrt(2.f*m) + sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;
    complex<float> root4 = -b/(4*a) + (-sqrt(2.f*m) - sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;

    vector<complex<float>> roots { root1, root2, root3, root4 };

    float max_real_root = 0.f;

    for (complex<float> root: roots)
    {
        if (root.imag() == 0)
        {
            max_real_root = max(max_real_root, root.real());
        }

        return max_real_root;
    }
}

void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans)
{
    int number_iter = 2;

    for (int i=0; i<number_iter; i++)
    {
        // Fill in the values for the matrices
        Eigen::MatrixXf M_i(2,4);
        Eigen::Matrix2f C_i;
        Eigen::MatrixXf n_i(2,1);
        Eigen::Vector2f pi_i;

        // Fill in the values for the matrices
        Eigen::Matrix4f M, W;
        Eigen::MatrixXf g(1,4);

        M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        g << 0, 0, 0, 0;

        for (int i=0; i < corresponds.size(); i++)
        {
            M_i << 1, 0, corresponds[i].p->getX(), -corresponds[i].p->getY(),
                    0, 1, corresponds[i].p->getY(), corresponds[i].p->getX();

            n_i = corresponds[i].getNormalNorm();

            C_i = n_i*n_i.transpose();

            M = M + M_i.transpose() * C_i * M_i;

            pi_i << corresponds[i].pj1->getX(), corresponds[i].pj1->getY();

            g = g + -2*pi_i.transpose() * C_i * M_i;
        }

        M = 2*M;

        // Define sub-matrices A, B, D from M
        Eigen::Matrix2f A, B, D;

        A << M.block<2,2>(0,0);
        B << M.block<2,2>(0,2);
        D << M.block<2,2>(2,2);

        // define S and S_A matriced from the matrices A, B and D
        Eigen::Matrix2f S;
        Eigen::Matrix2f S_A;

        S << D - (B.transpose() * A.inverse() * B);
        S_A << S.determinant() * S.inverse();

        // Get the coefficients for the lhs of the 2nd order equation
        Eigen::Matrix4f degree_2, degree_1, degree_0;

        degree_2 << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        degree_1 << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        degree_0 << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

        degree_2.topLeftCorner(2,2) = A.inverse() * B * B.transpose() * A.inverse().transpose();
        degree_2.topRightCorner(2,2) = -A.inverse() * B;
        degree_2.bottomLeftCorner(2,2) = -(A.inverse() * B).transpose();
        degree_2.bottomRightCorner(2,2).setIdentity();

        degree_1.topLeftCorner(2,2) = A.inverse() * B * S_A * B.transpose() * A.inverse().transpose();
        degree_1.topRightCorner(2,2) = -A.inverse() * B * S_A;
        degree_1.bottomLeftCorner(2,2) = -(A.inverse() * B * S_A).transpose();
        degree_1.bottomRightCorner(2,2) = S_A;

        degree_0.topLeftCorner(2,2) = A.inverse() * B * S_A.transpose() * S_A * B.transpose() * A.inverse().transpose();
        degree_0.topRightCorner(2,2) = -A.inverse() * B * S_A.transpose() * S_A;
        degree_0.bottomLeftCorner(2,2) = -(A.inverse() * B * S_A.transpose() * S_A);
        degree_0.bottomRightCorner(2,2) = S_A.transpose() * S_A;

        float lhs_pow2;
        float lhs_pow1;
        float lhs_pow0;

        lhs_pow2 = (4 * g * degree_2 * g.transpose())(0);
        lhs_pow1 = (4 * g * degree_1 * g.transpose())(0);
        lhs_pow0 = (g * degree_0 * g.transpose())(0);

        // Get the coefficients of the 4th order equation on rhs
        float pow4, pow3, pow2, pow1, pow0;

        pow4 = 16.0;
        pow3 = 16 * (S(0,0) + S(1,1));
        pow2 = 4*S(0,0)*S(0,0) + 4*S(1,1)*S(1,1) + 16*S(0,0)*S(1,1) - 8*S(0,1)*S(1,0) - lhs_pow2;
        pow1 = 4*S(0,0)*S(0,0)*S(1,1) + 4*S(0,0)*S(1,1)*S(1,1) - 4*S(0,0)*S(0,1)*S(1,0) - 4*S(0,1)*S(1,0)*S(1,1) - lhs_pow1;
        pow0 = S(0,0)*S(0,0)*S(1,1)*S(1,1) + S(0,1)*S(0,1)*S(1,0)*S(1,0) + 2*S(0,0)*S(0,1)*S(1,0)*S(1,1) - lhs_pow0;

        // double lambda = greatest_real_root(pow4, pow3, pow2, pow1, pow0);
        
        double x1, x2, x3, x4;

        int nbroots = solve_degree4(pow4, pow3, pow2, pow1, pow0, x1, x2, x3, x4);

        double lambda = x1>x2 ? x1:x2;
        lambda = lambda>x3 ? lambda:x3;
        lambda = lambda>x4 ? lambda:x4;

        cout << "                      lambda: " << lambda << "\n";

        Eigen::MatrixXf x(1,4);

        x = -((2*M + 2*lambda*W).inverse().transpose()) * g.transpose();

        double theta = atan2(x(3), x(2));

        curr_trans = Transform(x(0), x(1), theta);
    }
}
