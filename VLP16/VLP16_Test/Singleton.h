//
//  Singleton.h
//  PhDanceMasterApp
//
//  Created by YOHEI KAJIWARA on 2014/12/24.
//
//

#ifndef __Ainz_FaceMoverTest__Singleton__
#define __Ainz_FaceMoverTest__Singleton__

#include <stdio.h>

template <typename T>
class Singleton{
public:
    // 唯一のアクセス経路
    static inline T& GetInstance()
    {
        static T instance;  // 唯一のインスタンス
        return instance;
    }
    
protected:
    Singleton(){}
    virtual ~Singleton() {}
    
private:
    // 生成やコピーを禁止する
    Singleton(const Singleton& rhs);
    Singleton& operator=(const Singleton& rhs);
};

#endif /* defined(__Ainz_FaceMoverTest__Singleton__) */
