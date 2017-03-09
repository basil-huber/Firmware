/**
 * @file TargetTracker.hpp
 * Linear Discrete Kalman filter
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <stdint.h>




// M = number of states
// N = number of measurements
template<uint8_t M, uint8_t N>
class KalmanFilter
{

public:
	KalmanFilter(){};

	void init(matrix::SquareMatrix<float,M> f,
					matrix::Vector<float,M> w,
					matrix::Matrix<float,N,M> h,
					matrix::Vector<float,N> v,
					float dt)
	{
		_h = h;
	   	_h_t = _h.transpose();

		// Calculate q and psi
		matrix::Matrix<float,M,M> gwg;
		for(int i=0; i<M; i++)
			gwg(i,i) = w(i);
		matrix::SquareMatrix<float, 2*M> a;
		a.set(-f, 0,0);
		a.set(gwg,0,M);
		a.set(f.transpose(), M,M);
		a *= dt;

		matrix::SquareMatrix<float, 2*M> b = matrix::expm(a);
		for(int i=0; i<M; i++)
		{
			for(int j=0; j<M; j++)
			{
				_phi_t(i,j) = b(i+M,j+M);
				_q(i,j) = b(i,j+M);
			}
		}

		_phi = _phi_t.transpose();
		_q = _phi * _q;

		// set r
		for(int i=0; i<N; i++)
		{
			_r(i,i) = v(i);	
		}

		// set initial values for p and x
		_p.setIdentity();
		_p *= 100;	// TODO: this should be passed as paramenter to the init function
		_x.setZero();
	};


	void predict()
	{
		// perform prediction
		_x = _phi*_x;
		_p = _phi*_p*_phi_t + _q;
	};


	void correct(matrix::Vector<float,N> z)
	{
		// calc kalman gain k (weight/trust of measurement)
		matrix::SquareMatrix<float,N> a = (_h * _p * _h_t) + _r;
		matrix::Matrix<float,M,N> k = _p * _h_t * matrix::inv(a);
		
		// update state estimation
		_x = _x + (k*(z - (_h*_x)));

		// update estimation of state covariance
		matrix::SquareMatrix<float, M> i;
		i.setIdentity();
		_p = (i - (k*_h))*_p;
	}



	const matrix::Vector<float,M>& getStateEstimate() const {return _x;};

	matrix::Vector<float,M> getStateVariances() const
	{
		matrix::Vector<float,M> variances;

		for(int i= 0; i < M; i++)
		{
			variances(i) = _p(i,i);
		}
		return variances;
	}

private:
	matrix::SquareMatrix<float,M>   _q;			//< Covariance of system (model) noise [constant]
	matrix::SquareMatrix<float,N>   _r;			//< Covariance of measurement    noise [constant]
	matrix::Matrix<float,N,M>		_h;			//< measurement (design) matrix 	   [constant]
	matrix::SquareMatrix<float,M>   _phi;		//< state transition matrix 		   [constant]

	matrix::SquareMatrix<float,M> 	_p;   		//< A posteriori (estimated) state covariance
	matrix::Vector<float,M> 		_x;   		//< A posteriori (estimated) state

	// transposed matrices for faster calculation
	matrix::SquareMatrix<float,M> 	_phi_t;
	matrix::Matrix<float,M,N>		_h_t;
};
