/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#pragma once

#include <list>
#include "Iterable.hpp"

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>

////////////////////////////////////////////////////////////////////////
// std::list< std::reference_wrapper<E> > iterator implementation
////////////////////////////////////////////////////////////////////////

template<typename E>
class RefListIterator : public IteratorBase<E>
{
  public:

    RefListIterator(typename std::list< std::reference_wrapper<E> >::iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

    virtual E& operator * ()
    { return *it_; }

    const virtual E& operator * () const
    { return *it_; }

    virtual IteratorBase<E>* clone() const
    { return new RefListIterator( it_ ); }

  protected:

    virtual bool equal(const IteratorBase<E>& other) const
    { return it_ == ((RefListIterator&)other).it_; }

  private:

    typename std::list< std::reference_wrapper<E> >::iterator it_;
};

template<typename E>
class ConstRefListIterator : public ConstIteratorBase<E>
{
  public:

    ConstRefListIterator(typename std::list< std::reference_wrapper<E> >::const_iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

  const virtual E& operator * () const
    { return *it_; }

  protected:

    virtual bool equal(const ConstIteratorBase<E>& other) const
    {
      return it_ == ((ConstRefListIterator&)other).it_;
    }

    virtual ConstIteratorBase<E>* clone() const
    {
      return new ConstRefListIterator( it_ );
    }

  private:

    typename std::list< std::reference_wrapper<E> >::const_iterator it_;
};

template<typename E>
class RefListIterable : public IterableBase<E>
{
  public:

    virtual Iterator<E> begin()
    {
      return Iterator<E>( std::unique_ptr< RefListIterator<E> >( new RefListIterator<E>( container_.begin() ) ) );
    }

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.begin() ) ));
    }

    virtual Iterator<E> end()
    {
      return Iterator<E>( std::unique_ptr< RefListIterator<E> >( new RefListIterator<E>( container_.end() ) ) );
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static Iterable<E> from( std::list< std::reference_wrapper<E> >& container )
    {
      return Iterable<E>( std::unique_ptr< IterableBase<E> >( new RefListIterable<E>( container ) ) );
    }

  protected:

    RefListIterable( std::list< std::reference_wrapper<E> >& container ) : container_( container )
    {}

    std::list< std::reference_wrapper<E> >& container_;
};

template<typename E>
class ConstRefListIterable : public ConstIterableBase<E>
{
  public:

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.begin() ) ));
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static ConstIterable<E> from( const std::list< std::reference_wrapper<E> >& container )
    {
      return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new ConstRefListIterable<E>( container ) ) );
    }

  protected:

    ConstRefListIterable( const std::list< std::reference_wrapper<E> >& container ) : container_( container )
    {}

    const std::list< std::reference_wrapper<E> >& container_;
};

////////////////////////////////////////////////////////////////////////
// std::set< std::reference_wrapper<E> > iterator implementation
////////////////////////////////////////////////////////////////////////
/*
template<typename E, typename CMP>
class RefSetIterator : public IteratorBase<E>
{
  public:

    RefSetIterator(typename std::set< std::reference_wrapper<E>, CMP >::iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

    virtual E& operator * ()
    { return *it_; }

    const virtual E& operator * () const
    { return *it_; }

    virtual IteratorBase<E>* clone() const
    { return new RefSetIterator( it_ ); }

  protected:

    virtual bool equal(const IteratorBase<E>& other) const
    { return it_ == ((RefSetIterator&)other).it_; }

  private:

    typename std::set< std::reference_wrapper<E>, CMP >::iterator it_;
};

template<typename E, typename CMP>
class ConstRefSetIterator : public ConstIteratorBase<E>
{
  public:

    ConstRefSetIterator(typename std::set< std::reference_wrapper<E>, CMP >::const_iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

  const virtual E& operator * () const
    { return *it_; }

  protected:

    virtual bool equal(const ConstIteratorBase<E>& other) const
    {
      return it_ == ((ConstRefSetIterator&)other).it_;
    }

    virtual ConstIteratorBase<E>* clone() const
    {
      return new ConstRefSetIterator( it_ );
    }

  private:

    typename std::set< std::reference_wrapper<E>, CMP >::const_iterator it_;
};

template<typename E, typename CMP>
class RefSetIterable : public IterableBase<E>
{
  public:

    virtual Iterator<E> begin()
    {
      return Iterator<E>( std::unique_ptr< RefSetIterator<E, CMP> >( new RefSetIterator<E, CMP>( container_.begin() ) ) );
    }

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefSetIterator<E, CMP>( container_.begin() ) ));
    }

    virtual Iterator<E> end()
    {
      return Iterator<E>( std::unique_ptr< RefSetIterator<E, CMP> >( new RefSetIterator<E, CMP>( container_.end() ) ) );
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefSetIterator<E, CMP>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static Iterable<E> from( std::set< std::reference_wrapper<E>, CMP >& container )
    {
      return Iterable<E>( std::unique_ptr< IterableBase<E> >( new RefSetIterable<E, CMP>( container ) ) );
    }

  protected:

    RefSetIterable( std::set< std::reference_wrapper<E>, CMP >& container ) : container_( container )
    {}

    std::set< std::reference_wrapper<E>, CMP >& container_;
};

template<typename E, typename CMP>
class ConstRefSetIterable : public ConstIterableBase<E>
{
  public:

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefSetIterator<E, CMP>( container_.begin() ) ));
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefSetIterator<E, CMP>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static ConstIterable<E> from( const std::set< std::reference_wrapper<E>, CMP >& container )
    {
      return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new ConstRefSetIterable<E, CMP>( container ) ) );
    }

  protected:

    ConstRefSetIterable( const std::set< std::reference_wrapper<E>, CMP >& container ) : container_( container )
    {}

    const std::set< std::reference_wrapper<E>, CMP >& container_;
};
*/
////////////////////////////////////////////////////////////////////////
// std::list< std::shared_ptr<E> > iterator implementation
////////////////////////////////////////////////////////////////////////

template<typename E>
class SharedPtrListIterator : public IteratorBase<E>
{
  public:

    SharedPtrListIterator(typename std::list< std::shared_ptr<E> >::iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

    virtual E& operator * ()
    { return **it_; }

    const virtual E& operator * () const
    { return **it_; }

    virtual IteratorBase<E>* clone() const
    { return new SharedPtrListIterator( it_ ); }

  protected:

    virtual bool equal(const IteratorBase<E>& other) const
    { return it_ == ((SharedPtrListIterator&)other).it_; }

  private:

    typename std::list< std::shared_ptr<E> >::iterator it_;
};

template<typename E>
class ConstSharedPtrListIterator : public ConstIteratorBase<E>
{
  public:

    ConstSharedPtrListIterator(typename std::list< std::shared_ptr<E> >::const_iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

  const virtual E& operator * () const
    { return **it_; }

  protected:

    virtual bool equal(const ConstIteratorBase<E>& other) const
    {
      return it_ == ((ConstSharedPtrListIterator&)other).it_;
    }

    virtual ConstIteratorBase<E>* clone() const
    {
      return new ConstSharedPtrListIterator( it_ );
    }

  private:

    typename std::list< std::shared_ptr<E> >::const_iterator it_;
};

template<typename E>
class SharedPtrListIterable : public IterableBase<E>
{
  public:

    virtual Iterator<E> begin()
    {
      return Iterator<E>( std::unique_ptr< SharedPtrListIterator<E> >( new SharedPtrListIterator<E>( container_.begin() ) ) );
    }

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.begin() ) ));
    }

    virtual Iterator<E> end()
    {
      return Iterator<E>( std::unique_ptr< SharedPtrListIterator<E> >( new SharedPtrListIterator<E>( container_.end() ) ) );
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static Iterable<E> from( std::list< std::shared_ptr<E> >& container )
    {
      return Iterable<E>( std::unique_ptr< IterableBase<E> >( new SharedPtrListIterable<E>( container ) ) );
    }

  protected:

    SharedPtrListIterable( std::list< std::shared_ptr<E> >& container ) : container_( container )
    {}

    std::list< std::shared_ptr<E> >& container_;
};

template<typename E>
class ConstSharedPtrListIterable : public ConstIterableBase<E>
{
  public:

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.begin() ) ));
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static ConstIterable<E> from( const std::list< std::shared_ptr<E> >& container )
    {
      return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new ConstSharedPtrListIterable<E>( container ) ) );
    }

  protected:

    ConstSharedPtrListIterable( const std::list< std::shared_ptr<E> >& container ) : container_( container )
    {}

    const std::list< std::shared_ptr<E> >& container_;
};

////////////////////////////////////////////////////////////////////////
// std::set< std::shared_ptr<E> > iterator implementation
////////////////////////////////////////////////////////////////////////
/*
template<typename E, typename CMP>
class SharedPtrSetIterator : public IteratorBase<E>
{
  public:

    SharedPtrSetIterator(typename std::set< std::shared_ptr<E>, CMP >::iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

    virtual E& operator * ()
    { return **it_; }

    const virtual E& operator * () const
    { return **it_; }

    virtual IteratorBase<E>* clone() const
    { return new SharedPtrSetIterator( it_ ); }

  protected:

    virtual bool equal(const IteratorBase<E>& other) const
    { return it_ == ((SharedPtrSetIterator&)other).it_; }

  private:

    typename std::set< std::shared_ptr<E>, CMP >::iterator it_;
};

template<typename E, typename CMP>
class ConstSharedPtrSetIterator : public ConstIteratorBase<E>
{
  public:

    ConstSharedPtrSetIterator(typename std::set< std::shared_ptr<E>, CMP >::const_iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

  const virtual E& operator * () const
    { return **it_; }

  protected:

    virtual bool equal(const ConstIteratorBase<E>& other) const
    {
      return it_ == ((ConstSharedPtrSetIterator&)other).it_;
    }

    virtual ConstIteratorBase<E>* clone() const
    {
      return new ConstSharedPtrSetIterator( it_ );
    }

  private:

    typename std::set< std::shared_ptr<E>, CMP >::const_iterator it_;
};

template<typename E, typename CMP>
class SharedPtrSetIterable : public IterableBase<E>
{
  public:

    virtual Iterator<E> begin()
    {
      return Iterator<E>( std::unique_ptr< SharedPtrSetIterator<E, CMP> >( new SharedPtrSetIterator<E, CMP>( container_.begin() ) ) );
    }

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrSetIterator<E, CMP>( container_.begin() ) ));
    }

    virtual Iterator<E> end()
    {
      return Iterator<E>( std::unique_ptr< SharedPtrSetIterator<E, CMP> >( new SharedPtrSetIterator<E, CMP>( container_.end() ) ) );
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrSetIterator<E, CMP>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static Iterable<E> from( std::set< std::shared_ptr<E>, CMP >& container )
    {
      return Iterable<E>( std::unique_ptr< IterableBase<E> >( new SharedPtrSetIterable<E, CMP>( container ) ) );
    }

  protected:

    SharedPtrSetIterable( std::set< std::shared_ptr<E>, CMP >& container ) : container_( container )
    {}

    std::set< std::shared_ptr<E>, CMP >& container_;
};

template<typename E, typename CMP>
class ConstSharedPtrSetIterable : public ConstIterableBase<E>
{
  public:

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrSetIterator<E, CMP>( container_.begin() ) ));
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrSetIterator<E, CMP>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static ConstIterable<E> from( const std::set< std::shared_ptr<E>, CMP >& container )
    {
      return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new ConstSharedPtrSetIterable<E, CMP>( container ) ) );
    }

  protected:

    ConstSharedPtrSetIterable( const std::set< std::shared_ptr<E>, CMP >& container ) : container_( container )
    {}

    const std::set< std::shared_ptr<E>, CMP >& container_;
};
*/
////////////////////////////////////////////////////////////////////////

// push_back/emplace_back methods that return an iterator
// to the newly inserted element.
#define INSERT_BACK( ls, x ) (ls).insert( (ls).end(), x )
#define EMPLACE_BACK( ls, ... ) (ls).emplace( (ls).end(), __VA_ARGS__ )

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
class CovisibilityGraph
{
  public:
    // forward declarations
    class KeyFrame;
    class MapPoint;
    class Measurement;

    typedef std::shared_ptr<MapPoint> SharedMapPoint;
    typedef std::shared_ptr<KeyFrame> SharedKeyFrame;
    typedef std::shared_ptr<Measurement> SharedMeasurement;

  private:

    std::list<SharedKeyFrame> keyframes_;
    std::list<SharedMapPoint> mappoints_;

    struct compareSharedMapPoints {
       bool operator() (const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& p1, const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& p2) const
      { return p1.get() < p2.get(); }
    };

    struct compareSharedKeyFrames {
       bool operator() (const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame& k1, const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame& k2) const
      { return k1.get() < k2.get(); }
    };

    struct compareSharedMeasurements {
       bool operator() (const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMeasurement& m1, const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMeasurement& m2) const
      { return m1.get() < m2.get(); }
    };

  public:

    // This needs to be public because we need to use the same
    // comparison function in multiple places to search for duplicates
    // in a set :(
    typedef std::set<SharedMapPoint, compareSharedMapPoints> SharedMapPointSet;
    typedef std::set<SharedKeyFrame, compareSharedKeyFrames> SharedKeyFrameSet;
    typedef std::set<SharedMeasurement, compareSharedMeasurements> SharedMeasurementSet;

    typedef std::list<SharedKeyFrame> SharedKeyFrameList;
    typedef std::list<SharedMapPoint> SharedMapPointList;

    SharedKeyFrame addKeyFrame( const KEYFRAME_T& keyFrame );
    void removeKeyFrame( const SharedKeyFrame& keyFrame );

    SharedMapPoint addMapPoint( const MAP_POINT_T& mapPoint );
    void removeMapPoint( const SharedMapPoint& mapPoint );

    void addMeasurement( SharedKeyFrame& keyFrame, SharedMapPoint& mapPoint, const MEAS_T& edge );
    void removeMeasurement( const SharedMeasurement& edge );
//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - creo una nueva funcion removeMeasurement para poder utilizarla en tracker tomando como argumento el KF y MP que relaciona
    void removeMeasurement(  SharedKeyFrame& kf,  SharedMapPoint& mp);
//++++++++++++++++++++++++++++++++++++++++++++++++


    static inline ConstIterable<SharedKeyFrame> createIterable( SharedKeyFrameSet& set )
    { return ConstSetIterable<SharedKeyFrame, compareSharedKeyFrames>::from( set ); }

    SharedMapPointSet getLocalMap(/*const */KeyFrame& keyFrame);

    SharedKeyFrameList& getKeyframes()
    { return keyframes_; }

    SharedMapPointList& getMapPoints()
    { return mappoints_; }

  // Extensions for data classes

    class KeyFrame : public KEYFRAME_T, public std::enable_shared_from_this<KeyFrame>
    {
      public:

        KeyFrame( const KEYFRAME_T& elem ) : KEYFRAME_T( elem ) {}

        KeyFrame( const KeyFrame& other ) = delete; // non construction-copyable
        KeyFrame& operator=( const KeyFrame& ) = delete; // non copyable

        std::list< SharedMeasurement > measurements() const
        {
          boost::shared_lock<boost::shared_mutex> lock( meas_mutex_ );
          return  measurements_;
        }

//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - metodo que devuelve referencia a las meas de un KF, para poder ser recorridas por removeMeasurement() eliminando la que corresponda. En el caso de measurements() se devuelve una copia, por lo que el borrado no refleja el comportamiento deseado
        std::list< SharedMeasurement > & measurementsRef() const
        {
          boost::shared_lock<boost::shared_mutex> lock( meas_mutex_ );
          return  measurements_;
        }
//++++++++++++++++++++++++++++++++++++++++++++++++

        std::map< SharedKeyFrame, size_t, compareSharedKeyFrames> covisibilityKeyFrames()
        {
          boost::shared_lock<boost::shared_mutex> lock( covisibilityKeyframes_mutex_ );
          return covisibilityKeyFrames_;
        }

      private:

        void addCovisibilityKeyframe(SharedKeyFrame covisibilityKeyFrame) {
            boost::unique_lock<boost::shared_mutex> lock( covisibilityKeyframes_mutex_ );

            auto it = covisibilityKeyFrames_.find( covisibilityKeyFrame );

            if (it != covisibilityKeyFrames_.end())
              it->second++;   // Increase the covisibility value if the edge already exists
            else // was not found, create covisibility edge if it does not exists
              covisibilityKeyFrames_.insert(std::pair<CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame,size_t>(covisibilityKeyFrame, 1));
        }

        void addMeasurement(SharedMeasurement& meas) {
          boost::unique_lock<boost::shared_mutex> lock( meas_mutex_ );
          meas->it_keyFrame_ = INSERT_BACK(measurements_, meas);
        }

        mutable boost::shared_mutex meas_mutex_;

        mutable boost::shared_mutex covisibilityKeyframes_mutex_;

        mutable std::list< SharedMeasurement > measurements_;

        mutable std::map< SharedKeyFrame, size_t, compareSharedKeyFrames> covisibilityKeyFrames_;

        void setIteratorToContainer(const typename std::list<SharedKeyFrame>::iterator& it){ to_container_ = it; }

        typename std::list<SharedKeyFrame>::iterator to_container_;

        friend class CovisibilityGraph;
    };

    class MapPoint : public MAP_POINT_T
    {
       public:

        MapPoint( const MAP_POINT_T& elem ) : MAP_POINT_T( elem ) {}

        MapPoint( const MapPoint& other ) = delete; // non construction-copyable
        MapPoint& operator=( const MapPoint& ) = delete; // non copyable

        std::list< SharedMeasurement > measurements() const
        {
          boost::shared_lock<boost::shared_mutex> lock( meas_mutex_ );
          return measurements_;
        }


//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM -  metodo que devuelve referencia a las meas de un MP, para poder ser recorridas por removeMeasurement() eliminando la que corresponda. En el caso de measurements() se devuelve una copia, por lo que el borrado no refleja el comportamiento deseado
        std::list< SharedMeasurement > & measurementsRef() const
        {
          boost::shared_lock<boost::shared_mutex> lock( meas_mutex_ );
          return  measurements_;
        }
//++++++++++++++++++++++++++++++++++++++++++++++++


      private:

        void addMeasurement(SharedMeasurement& meas) {
          boost::unique_lock<boost::shared_mutex> lock( meas_mutex_ );
          meas->it_mapPoint_ = INSERT_BACK(measurements_, meas);
        }

        mutable boost::shared_mutex meas_mutex_;

        mutable std::list< SharedMeasurement > measurements_;

        void setIteratorToContainer(const typename std::list<SharedMapPoint>::iterator& it){ to_container_ = it; }

        typename std::list<SharedMapPoint>::iterator to_container_;

        friend class CovisibilityGraph;
    };

    class Measurement : public MEAS_T
    {
      public:

        Measurement(const MEAS_T& edge, SharedKeyFrame& keyFrame, SharedMapPoint& mapPoint)
          : MEAS_T( edge ), keyFrame_( keyFrame ), mapPoint_( mapPoint ) {}

        Measurement( const Measurement& other ) = delete; // non construction-copyable
        Measurement& operator=( const Measurement& ) = delete; // non copyable

        inline SharedKeyFrame& keyFrame()
        { return keyFrame_; }

        inline const SharedKeyFrame& keyFrame() const
        { return keyFrame_; }

        inline SharedMapPoint& mapPoint()
        { return mapPoint_; }

        inline const SharedMapPoint& mapPoint() const
        { return mapPoint_; }

      private:

        SharedKeyFrame keyFrame_;
        typename std::list< SharedMeasurement >::/*const_*/iterator it_keyFrame_;

        SharedMapPoint mapPoint_;
        typename std::list< SharedMeasurement >::/*const_*/iterator it_mapPoint_;

        friend class CovisibilityGraph;
    };
};

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addKeyFrame( const KEYFRAME_T& keyFrame )
{
  SharedKeyFrame sKF(new KeyFrame( keyFrame ));

  keyframes_.push_back(sKF);

  sKF->setIteratorToContainer(std::prev(keyframes_.end()));

  return sKF;
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeKeyFrame( const CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame& keyFrame )
{
  while ( not keyFrame->measurements_.empty() )
    removeMeasurement( keyFrame->measurements_.front() );

    keyframes_.erase(keyFrame->to_container_);
    keyFrame->to_container_ = keyframes_.end();
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addMapPoint( const MAP_POINT_T& mapPoint )
{
  SharedMapPoint sMP(new MapPoint( mapPoint ));

  mappoints_.push_back(sMP);

  sMP->setIteratorToContainer(std::prev(mappoints_.end()));

  return sMP;
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeMapPoint( const CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& mapPoint )
{
  while ( not mapPoint->measurements_.empty() )
    removeMeasurement( mapPoint->measurements_.front() );

    mappoints_.erase(mapPoint->to_container_);
    mapPoint->to_container_ = mappoints_.end();
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addMeasurement( SharedKeyFrame& keyFrame, SharedMapPoint& mapPoint, const MEAS_T& edge )
{

  /* Adding measurements to keyframes or mappoints that aren't in the graph, are discarted */
  if(keyFrame->to_container_ == keyframes_.end() or mapPoint->to_container_ == mappoints_.end())
    return;

  // Check if the point is in the graph

  // Get Keyframes which measure the point

  // This is a copy
  std::list< SharedMeasurement > point_measurements = mapPoint->measurements();

  for ( SharedMeasurement& meas_point : point_measurements )
  {
    // Omit current keyframe
    if (meas_point->keyFrame().get() == keyFrame.get())
      continue;

    CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame covisibilityKeyFrame ( meas_point->keyFrame() );

    // TODO: Gaston: La operacion no es atomica, por lo que un keyframe podria ser covisible a otro y no viceversa.
    keyFrame->addCovisibilityKeyframe( covisibilityKeyFrame );
    covisibilityKeyFrame->addCovisibilityKeyframe( keyFrame );


//    std::cout << "Keyframe: " << ( meas_point->keyFrame() ) << " nivel de covisibilidad: " << keyFrame->covisibilityKeyFrames_.at( covisibilityKeyFrame )  << std::endl;
//    std::cout << "covisibilityKeyFrames size: " << keyFrame->covisibilityKeyFrames_.size() << std::endl;
  }

  // add Keyframe-Point Measurement

  SharedMeasurement meas( new Measurement(edge, keyFrame, mapPoint) );
  // TODO: make atomic this behavior otherwise a keyframe may be connected to the mappoint but not yet the mappoint to the keyframe (concurrency)
  keyFrame->addMeasurement(meas);
  mapPoint->addMeasurement(meas);
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeMeasurement( const SharedMeasurement& edge )
{
  /* TODO: Gaston: Covisibility information of the keyframe involved must be updated on removal */
  edge->keyFrame_->measurements_.erase( edge->it_keyFrame_ );
  edge->mapPoint_->measurements_.erase( edge->it_mapPoint_ );
}


//++++++++++++++++++++++++++++++++++++++++++++++++
// DSPTAM - creo una nueva funcion removeMeasurement para poder utilizarla en tracker tomando como argumento el KF y MP que relaciona
template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeMeasurement(  SharedKeyFrame& kf,  SharedMapPoint& mp)
{
 
// Elimino Meas del MP y del KF

// busco dentro de los KF el measurement que lo vincula con el MP
  for (auto it = kf->measurementsRef().begin(); it != kf->measurementsRef().end(); it++ )
    if ( (*it)->mapPoint()->getMapPointId() == mp->getMapPointId() )
    {
      kf->measurementsRef().erase(it);
      break;
    }

// busco dentro de los MP el measurement que lo vincula con el KF
  for (auto it = mp->measurementsRef().begin(); it != mp->measurementsRef().end(); it++ )
    if ( (*it)->keyFrame()->GetId() == kf->GetId() )
    {
      mp->measurementsRef().erase(it);
      break;
    }

}
//++++++++++++++++++++++++++++++++++++++++++++++++


//template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
//inline bool operator< (const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& p1, const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& p2)
//{ return &(p1.get()) < &(p2.get()); }

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPointSet CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::getLocalMap(/*const */CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::KeyFrame& keyFrame)
{
  typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPointSet localMap;

  // Add points viewed by the query keyFrame

  // This is a copy
  std::list< SharedMeasurement > measurements = keyFrame.measurements();

  for(auto& meas : measurements)
  {
    const CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint mapPoint ( meas->mapPoint() );
    localMap.insert( mapPoint );
  }

  // Also add other points viewed by the set of covisible keyFrames.

  auto covisibilityKeyFrames = keyFrame.covisibilityKeyFrames();

  for (auto& covisibilitykeyframe : covisibilityKeyFrames )
  {
    for ( auto& meas : covisibilitykeyframe.first->measurements() )
    {
      const CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint mapPoint ( meas->mapPoint() );
      localMap.insert( mapPoint );
    }
  }

  return localMap;
}
