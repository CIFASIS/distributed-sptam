#include <vector>
#include <memory>

namespace dsptam {


typedef std::size_t id;

template <class T>
class Translations
{
  public:
      Translations(id n) : vector(n) {};
      inline void set(const id n, const T ref)
       { vector[n] = ref; }

      inline T & get(const id n)
       { return vector[n]; }

       inline size_t size()
       { return vector.size(); }

       inline size_t are_set()    
       { size_t n = 0;
       	 for (auto i: vector)
       	 	if (i == 0)
       	 		n++;
       	 return vector.size() - n;
       }

       inline bool idIsNotSet(id n)
       { return vector[n] == nullptr; } 

  private:

      std::vector< T > vector;
};


typedef Translations<sptam::Map::SharedPoint> TranslationsMapPoint;
typedef Translations<sptam::Map::SharedKeyFrame> TranslationsKeyFrame;
} // namespace




