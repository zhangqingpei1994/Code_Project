/****************************************************************************
** Meta object code from reading C++ file 'mythread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mythread/mythread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mythread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_forceThread_t {
    QByteArrayData data[7];
    char stringdata0[59];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_forceThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_forceThread_t qt_meta_stringdata_forceThread = {
    {
QT_MOC_LITERAL(0, 0, 11), // "forceThread"
QT_MOC_LITERAL(1, 12, 10), // "sandsignal"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 7), // "double*"
QT_MOC_LITERAL(4, 32, 6), // "signal"
QT_MOC_LITERAL(5, 39, 10), // "robotstate"
QT_MOC_LITERAL(6, 50, 8) // "q_actual"

    },
    "forceThread\0sandsignal\0\0double*\0signal\0"
    "robotstate\0q_actual"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_forceThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x06 /* Public */,
       4,    1,   32,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   35,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, QMetaType::QString,    2,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    6,

       0        // eod
};

void forceThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        forceThread *_t = static_cast<forceThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sandsignal((*reinterpret_cast< double*(*)>(_a[1]))); break;
        case 1: _t->signal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->robotstate((*reinterpret_cast< double*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (forceThread::*_t)(double * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&forceThread::sandsignal)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (forceThread::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&forceThread::signal)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject forceThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_forceThread.data,
      qt_meta_data_forceThread,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *forceThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *forceThread::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_forceThread.stringdata0))
        return static_cast<void*>(const_cast< forceThread*>(this));
    return QThread::qt_metacast(_clname);
}

int forceThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void forceThread::sandsignal(double * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void forceThread::signal(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
