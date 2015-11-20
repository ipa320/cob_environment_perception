#ifndef SPHERE_SAMPLER_INCLUDED
#define SPHERE_SAMPLER_INCLUDED

#include <omp.h>
#include "ShapeSPH/Util/Geometry.h"
//#include "ShapeSPH/SignalProcessing/CubeGrid.h"
#include "ShapeSPH/SignalProcessing/SphericalGrid.h"
#include "ShapeSPH/SignalProcessing/Fourier.h"

///replacement for vector which allows resizing without initialization
template<class T>
class uninit_vector {
	T *data_;
	size_t s_;
	
public:
	uninit_vector():data_(0), s_(0) {}
	~uninit_vector() {delete data_;}
	uninit_vector(const uninit_vector &o) {
		data_ = new T[o.s_];
		s_=o.s_;
		//memcpy(data_, o.data_, s_);
		for(size_t i=0; i<s_; i++)
			data_[i] = o.data_[i];
	}

	void operator=(const uninit_vector &o) {
		resize(o.s_);
		//memcpy(data_, o.data_, s_);
		for(size_t i=0; i<s_; i++)
			data_[i] = o.data_[i];
	}
	
	inline size_t size() const {return s_;}
	inline void resize(const size_t s) {delete data_; data_ = new T[s];s_=s;}
	inline void resize(const size_t s, const T &v) {delete data_; data_ = new T[s];s_=s;for(size_t i=0; i<s; i++) data_[i]=v;}
	
	inline T &operator[](const size_t i) {return data_[i];}
	inline const T &operator[](const size_t i) const {return data_[i];}
};

template< class Real, class RealSampling >
struct Sampler {
	typedef std::vector< uninit_vector<Real> > RealValues;
	typedef std::vector< uninit_vector<std::complex<RealSampling> > > Values;
	typedef std::vector< uninit_vector<Eigen::Matrix<RealSampling, 3,1> > > Samples;

	Real maxRadius_;
	const int radii_, sphereResolution_;
	RealValues vals_;
	Values complex_vals_;

	Sampler(Real maxRadius , int radii , int sphereResolution) : maxRadius_(maxRadius), radii_(radii), sphereResolution_(sphereResolution)
	{
		initValues(complex_vals_);
		clear();
	}
	
	void setMaxRadius(const Real maxRadius) {maxRadius_=maxRadius;}
	
	void clear() {
		for( int i=0 ; i<radii_ ; i++ )
			for(size_t j=0; j<complex_vals_[i].size(); j++)
				complex_vals_[i][j]=0;
	}

	void initValues(Values &vals) {
		vals.resize(radii_);
		for( int i=0 ; i<radii_ ; i++ )
			vals[i].resize(sphereResolution_*sphereResolution_/2);
	}

	void operator+=(const Values &vals) {
		assert((int)vals.size()==radii_);
		for( int i=0 ; i<radii_ ; i++ ) {
			assert(vals[i].size()==complex_vals_[i].size());
			for(size_t j=0; j<vals[i].size(); j++)
				complex_vals_[i][j] += vals[i][j];
		}
	}

	template<class R>
	R radius(const int i) const {
		//if(i<8)
		//return ( R(i)/radii_ ) * maxRadius_ + R(std::pow(1.5, i));
		//return R(std::pow(2, i));
		/*std::cout
		<<"1. "<<R(std::pow(1.7, i)*maxRadius_/std::pow(1.7, radii_))<<std::endl
		<<"2. "<<R(( R(i+1.)/radii_ ) * maxRadius_)<<std::endl
		<<"3. "<<R(std::pow(1.7, i))<<std::endl;*/
		//return R(std::pow(1.7, i)*maxRadius_/std::pow(1.7, radii_));//( R(i+1.)/radii_ ) * maxRadius_;
		return ( R(i+1.)/radii_ ) * maxRadius_;
		return R(std::pow(1.7, i));//( R(i+1.)/radii_ ) * maxRadius_;
	}

	int index(const int& i,const int& j) const {
		int x=j,y=i;

		if(x<0){x=2*sphereResolution_-((-i)%(2*sphereResolution_));}
		x%=2*sphereResolution_;
		if(x>=sphereResolution_){
			x=2*sphereResolution_-x-1;
			y+=sphereResolution_/2;
		}
		if(y<0){y=sphereResolution_-((-y)%sphereResolution_);}
		y=y%sphereResolution_;
		return x*sphereResolution_+y;
	}

	void getSamples(Samples &samples, int threads=1) const
	{
		samples.resize(radii_);
		for(int l=0 ; l<radii_; l++ )
			samples[l].resize(sphereResolution_*sphereResolution_/2);

		//#pragma omp parallel for num_threads( threads )
		for( int i=0 ; i<sphereResolution_ ; i++ ) for( int j=0 ; j<sphereResolution_/2; j++ )
		{
			Eigen::Matrix<RealSampling, 3,1> v;
			//TODO: check if this is valid for our case
			//OLD: const RealSampling theta = 2.0*PI*i/sphereResolution_;
			//const RealSampling theta = PI*i/sphereResolution_;
			
			const RealSampling theta = 2.0*PI*i/sphereResolution_;
			const RealSampling phi = PI*(2.0*j+1)/(2.0*sphereResolution_);
			v(0)=std::sin(phi)*std::cos(theta);
			v(1)=std::cos(phi);
			v(2)=std::sin(phi)*std::sin(theta);

			for(int l=0 ; l<radii_; l++ )
				samples[l][index(i,j)] = v*radius<RealSampling>(l);
		}
	}

	void abs() {
		//Samples s;
		//getSamples(s);
		
		vals_.resize(radii_);
		for( int i=0 ; i<radii_ ; i++ ) {
			vals_[i].resize(sphereResolution_*sphereResolution_);
			const RealSampling scale = RealSampling( sqrt( 4*M_PI*std::pow(radius<RealSampling>(i),2) ) );
			for(size_t j=0; j<complex_vals_[i].size(); j++) {
				vals_[i][j] = std::abs(complex_vals_[i][j])*scale;
				//vals_[i][j] = (std::abs(complex_vals_[i][j])+std::arg(complex_vals_[i][j]))*scale;
				
				vals_[i][(sphereResolution_*2-1-(j/(sphereResolution_/2)))*sphereResolution_/2+(j%(sphereResolution_/2))] = vals_[i][j];
				//if(j<vals_[i].size()/2)
				//std::cout<<std::abs(complex_vals_[i][j])<<" -> "<<std::abs(complex_vals_[i][(sphereResolution_*2-1-(j/(sphereResolution_/2)))*sphereResolution_/2+(j%(sphereResolution_/2))])<<std::endl;
			}
		}
		
		/*for( int l=0 ; l<radii_ ; l++ ) {
			for( int i=0 ; i<sphereResolution_ ; i++ ) {
				for( int j=0 ; j<sphereResolution_; j++ )
				{
					//if(i>=sphereResolution_/2) vals_[l][index(i,j)]=0;
					//vals_[l][index(i,j)] = rand()%100;
					//std::cout<<s[l][index(i,j)].transpose()<<" ";
					std::cout<<complex_vals_[l][index(i,j)]<<" ";
					std::cout<<vals_[l][index(i,j)]<<" \t";//<<index(i,j)<<"\t ";
				}
				std::cout<<std::endl;
			}
			std::cout<<std::endl;
		}*/
	}

	void sample(std::vector< FourierKeyS2< Real > >& sphericalHarmonics, int threads=1) {
		abs();

		HarmonicTransform< Real > xForm( sphereResolution_ );
		SphericalGrid< Real > sphere( sphereResolution_ );
		sphericalHarmonics.resize( radii_ );
		for( int i=0 ; i<radii_ ; i++ )
		{
			sphericalHarmonics[i].resize( sphereResolution_ );
			xForm.ForwardFourier(&vals_[i][0], sphereResolution_, sphericalHarmonics[i] );
		}
	}
	
	void finish(Signature< Real > &sig) {
		std::vector< FourierKeyS2< Real > > sKeys;
		sample(sKeys);
		
		if(sKeys.size()<1) return;
		
		const int bw = sKeys[0].bandWidth();
		sig.resize( (bw) * int( sKeys.size() ) );
		for( size_t i=0 ; i<sKeys.size() ; i++ )
			for( int b=0 ; b<bw ; b++ )
			{
				Real _norm2 = sKeys[i](b,0).squareNorm();
				for( int j=1 ; j<=b ; j++ )
					_norm2 += sKeys[i](b,j).squareNorm()*2;
				sig[i*bw+b] = Real( sqrt(_norm2) )/(i+1);
			}
	}
};

#endif // SPHERE_SAMPLER_INCLUDED
