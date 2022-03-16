//
// Created by yue on 15/3/2022.
//


# include <iostream>        // standard input/output
# include <vector>          // standard vector
# include <cppad/cppad.hpp> // the CppAD package


namespace { // begin the empty namespace
    //获取组合多项式
    // define the function Poly(a, x) = a[0] + a[1]*x[1] + ... + a[k-1]*x[k-1]
    //f(x)=a_{1}+a_{1}∗x^(1)+⋯+a_{k-1}∗x^(k-1)
    template <class Type>
    Type Poly(const CPPAD_TESTVECTOR(double) &a, const Type &x,const Type & flag_exp)
    {   size_t k  = a.size();
        Type y   = 0.;  // initialize summation
        Type x_i = 1.;  // initialize x^i
        if (flag_exp>=1){
            for(size_t i = 0; i < k; i++)
            {   y   += a[i] * x_i;  // y   = y + a_i * x^i
                x_i *= x;           // x_i = x_i * x
            }
        }else
        {
            //f(x)=exp(x)−1−x−x^(2)/2
            //a0=-1.a1=-1.a2=-1/2
            y= exp(x);
            for(size_t i = 0; i < k; i++)
            {
                y   += a[i] * x_i;  // y   = y + a_i * x^i
                x_i *= x;           // x_i = x_i * x
            }
        }
        return y;
    }
}
int main(int argc, char** argv)
{
    using CppAD::AD;
    using std::vector;
    if (argc != 2) {
        std::cout<<"Please choose the example num:"<<std::endl;
        return EXIT_FAILURE;
    }
    if (argv[1][0]=='1') {
        size_t k = 5;
        CPPAD_TESTVECTOR(double) a(k);
        for (int i = 0; i < k; ++i) {
            a[i] = 1.0;// value of polynomial coefficients
        }
        size_t n = 1;
        vector<AD<double>> ax(n);// vector of domain space variables
        ax[0] = 2;// value at which function is recorded
        // declare independent variables and start recording operation sequence
        CppAD::Independent(ax);
        // range space vector
        size_t m = 1;               // number of ranges space variables
        vector<AD<double> > ay(m); // vector of ranges space variables
        vector<AD<double>> a_flag(1);
        a_flag[0] = 1;
        ay[0] = Poly(a, ax[0], a_flag[0]);     // record operations that compute ay[0]
        std::cout << "f(x=3,y)" << ay[0] << std::endl;
        // store operation sequence in f: X -> Y and stop recording
        CppAD::ADFun<double> f(ax, ay);
        // compute derivative using operation sequence stored in f
        vector<double> jac(m * n); // Jacobian of f (m by n matrix)
        vector<double> x(n);       // domain space vector
        x[0] = 2.;                 // argument value for computing derivative
        jac = f.Jacobian(x);      // Jacobian for operation sequence
        // print the results
        std::cout << "f'(3) computed by CppAD = " << jac[0] << std::endl;
        // check if the derivative is correct
        int error_code;
        if (jac[0] == 49.)
            error_code = 0;      // return code for correct case
        else error_code = 1;    // return code for incorrect case
        return error_code;
    }else if(argv[1][0]=='2')
    {
        size_t k = 3;
        CPPAD_TESTVECTOR(double) a(k);
        a[0] = 1.0;
        a[1] = 1.0;
        a[2] = 1.0/2;
        size_t n = 1;
        vector<AD<double>> ax(n);// vector of domain space variables
        ax[0] = 0.5;// value at which function is recorded
        // declare independent variables and start recording operation sequence
        CppAD::Independent(ax);
        // range space vector
        size_t m = 1;               // number of ranges space variables
        vector<AD<double> > ay(m); // vector of ranges space variables
        vector<AD<double>> a_flag(1);
        a_flag[0] = 1;
        ay[0] = Poly(a, ax[0], a_flag[0]);     // record operations that compute ay[0]
        std::cout << "f(x=0.5,y)" << ay[0] << std::endl;
        // store operation sequence in f: X -> Y and stop recording
        CppAD::ADFun<double> f(ax, ay);
        // compute derivative using operation sequence stored in f
        vector<double> jac(m * n); // Jacobian of f (m by n matrix)
        vector<double> x(n);       // domain space vector
        x[0] = 0.5;                 // argument value for computing derivative
        jac = f.Jacobian(x);      // Jacobian for operation sequence
        // print the results
        std::cout << "f'(0.5) computed by CppAD = " << jac[0] << std::endl;
        // check if the derivative is correct
        int error_code;
        if (jac[0] == 1.5)
            error_code = 0;      // return code for correct case
        else error_code = 1;    // return code for incorrect case
        return error_code;
    }else if(argv[1][0]=='3')
    {
        size_t k = 3;
        CPPAD_TESTVECTOR(double) a(k);
        a[0] = -1.0;
        a[1] = -1.0;
        a[2] = -1.0/2;
        size_t n = 1;
        vector<AD<double>> ax(n);// vector of domain space variables
        ax[0] = 0.5;// value at which function is recorded
        // declare independent variables and start recording operation sequence
        CppAD::Independent(ax);
        // range space vector
        size_t m = 1;               // number of ranges space variables
        vector<AD<double> > ay(m); // vector of ranges space variables
        vector<AD<double>> a_flag(1);
        a_flag[0] = 0;
        ay[0] = Poly(a, ax[0], a_flag[0]);     // record operations that compute ay[0]
        std::cout << "f(x=0.5,y)" << ay[0] << std::endl;
        // store operation sequence in f: X -> Y and stop recording
        CppAD::ADFun<double> f(ax, ay);
        // compute derivative using operation sequence stored in f
        vector<double> jac(m * n); // Jacobian of f (m by n matrix)
        vector<double> x(n);       // domain space vector
        x[0] = 0.5;                 // argument value for computing derivative
        jac = f.Jacobian(x);      // Jacobian for operation sequence
        // print the results
        std::cout << "f(x)=exp(x)−1−x−x^(2)/2, f'(0.5) computed by CppAD = " << jac[0] << std::endl;
        // check if the derivative is correct
        int error_code;
        if (jac[0] ==  0.148721)
            error_code = 0;      // return code for correct case
        else error_code = 1;    // return code for incorrect case
        return error_code;
    }
}