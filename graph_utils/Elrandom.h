#ifndef __ELRANDOM_HH
#define __ELRANDOM_HH

#include "Header.h"


//EasyLocal Random (elrandom.hh)

class ElRandom
{
public:
  /** Generates an uniform random integer in [a, b].
   \param a lower bound
    \param b upper bound
    */
  template <typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
  static T Uniform(T a, T b) {
    std::uniform_int_distribution<T> d(a, b);
    return d(GetInstance().g);
  }
  
  /** Generates an uniform random float in [a, b].
   \param a lower bound
    \param b upper bound
    */
  template <typename T, typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
  static T Uniform(T a, T b) {
    std::uniform_real_distribution<T> d(a, b);
    return d(GetInstance().g);
  }

  /** Generates a random float according to the Lomax distribution, whose density function is X = lambda * ((1/(1-U)^(1/alpha)) - 1), 
  *     where U is a uniformly distributed random variable in [0,1)
  \param lambda > 0 : double scale factor
   \param alpha > 0 : double shape  
   */
  template <typename T, typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
  static T Lomax(T scale, T shape) {
      assert(scale > EPSILON && shape > EPSILON);
      std::uniform_real_distribution<T> d(0.0, 1.0 - EPSILON);
      double base = 1.0 - d(GetInstance().g);
      double exponent = 1.0 / shape;
      double lomax_number = ((1.0 / pow(base, exponent)) - 1) * scale;
      return lomax_number;
  }

  /** Generates a random integer according to the discrete distribution according to the weights W in parameter
  *     P(i|W_0,...,W_{n-1}) = W_i/sum_{k=0,...,n-1} W_k, for each i=0,...,n-1; n is the size of W
   *   \param weights
    */
  template <typename T>
  static int Discrete(const vector<T>& weights) {
      std::discrete_distribution<int> d(weights.begin(), weights.end());
      return d(GetInstance().g);
  }

  /** Generates randomly true or false with probability p and 1-p respectively.
   *   \param p
    */
  //template <typename T>
  static int Bernoulli(const double& p) {
      std::bernoulli_distribution d(p);
      return d(GetInstance().g);
  }

  /** Generates a random integer according to the binomial distribution according to probability of success p in the range [0,t]
  *     P(i|t,p) = (t i) p^i(1-p)^{t-i} 
   *   \param p
   *   \param t
    */
  //template <typename T>
  static int Binomial(const int& t, const double& p) {
      std::binomial_distribution<int> d(t,p);
      return d(GetInstance().g);
  }
  
  /** Sets a new seed for the random engine. */
  static unsigned int SetSeed(unsigned int seed)
  {
    ElRandom& r = GetInstance();
    r.g.seed(seed);
    return r.seed = seed;
  }
  
  static unsigned int GetSeed()
  {
    return GetInstance().seed;
  }
  
  
  static std::mt19937& GetGenerator()
  {
    return GetInstance().g;
  }
  
private:
  static ElRandom& GetInstance() {
    static ElRandom instance;
    return instance;
  }
  
  ElRandom()
  {
    std::random_device dev;
    seed = dev();
    g.seed(seed);
  }
  
  std::mt19937 g;
  
  unsigned int seed;
};

#endif