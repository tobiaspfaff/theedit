#include "vectors.hpp"
#include "util.hpp"
using namespace std;

template <int n, typename T> Mat<n,n,T> inverse (const Mat<n,n,T> &A) {
    //cout << "warning: unoptimized " << n << "x" << n << " matrix inversion." << endl;
/*    real_2d_array a;
    a.setlength(n,n);
    for (int i=0; i<n; i++)
        for(int j=0; j<n; j++)
            a(i,j) = A(i,j);

    ae_int_t info;
    matinvreport rep;
    rmatrixinverse(a, info, rep);
    
    if (info < 0)
        ERROR("matrix inversion failed", ERR_LOGIC);

    Mat<n,n,T> B;
    for (int i=0; i<n; i++)
        for(int j=0; j<n; j++)
            B(i,j) = a(i,j);
    return B;*/
    ERROR("general inverse not supported");
}

template<> Mat2x2 inverse(const Mat2x2& A) {
    return Mat2x2(Vec2(A(1,1), -A(1,0)), Vec2(-A(0,1), A(0,0)))/det(A);
}
template<> Mat3x3 inverse(const Mat3x3& A) {
    return Mat3x3(cross(A.col(1),A.col(2)), cross(A.col(2),A.col(0)), cross(A.col(0),A.col(1))).t()/det(A);
}


Mat2x2 getRotMatrix(double rotation, const Vec2& scale) {
    Mat2x2 R(Vec2(cos(rotation),sin(rotation)),
             Vec2(-sin(rotation),cos(rotation)));
    R *= Mat2x2(Vec2(scale[0],0),Vec2(0,scale[1]));
    return R;
}
