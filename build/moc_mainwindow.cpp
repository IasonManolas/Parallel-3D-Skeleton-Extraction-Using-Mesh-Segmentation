/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[21];
    char stringdata0[450];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 11), // "modelLoaded"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 11), // "std::string"
QT_MOC_LITERAL(4, 36, 8), // "filename"
QT_MOC_LITERAL(5, 45, 24), // "stateCheckBoxAxesChanged"
QT_MOC_LITERAL(6, 70, 5), // "state"
QT_MOC_LITERAL(7, 76, 31), // "stateCheckBoxMeshSurfaceChanged"
QT_MOC_LITERAL(8, 108, 24), // "buttonResetCameraClicked"
QT_MOC_LITERAL(9, 133, 32), // "stateCheckBoxShowVerticesChanged"
QT_MOC_LITERAL(10, 166, 24), // "actionSaveModelTriggered"
QT_MOC_LITERAL(11, 191, 11), // "destination"
QT_MOC_LITERAL(12, 203, 26), // "actionSaveSegmentTriggered"
QT_MOC_LITERAL(13, 230, 23), // "on_checkBoxAxis_clicked"
QT_MOC_LITERAL(14, 254, 27), // "on_actionOpenFile_triggered"
QT_MOC_LITERAL(15, 282, 30), // "on_checkBoxMeshSurface_clicked"
QT_MOC_LITERAL(16, 313, 32), // "on_pushButtonResetCamera_clicked"
QT_MOC_LITERAL(17, 346, 36), // "on_checkBoxShowVertices_state..."
QT_MOC_LITERAL(18, 383, 4), // "arg1"
QT_MOC_LITERAL(19, 388, 29), // "on_actionSave_Model_triggered"
QT_MOC_LITERAL(20, 418, 31) // "on_actionSave_Segment_triggered"

    },
    "MainWindow\0modelLoaded\0\0std::string\0"
    "filename\0stateCheckBoxAxesChanged\0"
    "state\0stateCheckBoxMeshSurfaceChanged\0"
    "buttonResetCameraClicked\0"
    "stateCheckBoxShowVerticesChanged\0"
    "actionSaveModelTriggered\0destination\0"
    "actionSaveSegmentTriggered\0"
    "on_checkBoxAxis_clicked\0"
    "on_actionOpenFile_triggered\0"
    "on_checkBoxMeshSurface_clicked\0"
    "on_pushButtonResetCamera_clicked\0"
    "on_checkBoxShowVertices_stateChanged\0"
    "arg1\0on_actionSave_Model_triggered\0"
    "on_actionSave_Segment_triggered"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   84,    2, 0x06 /* Public */,
       5,    1,   87,    2, 0x06 /* Public */,
       7,    1,   90,    2, 0x06 /* Public */,
       8,    0,   93,    2, 0x06 /* Public */,
       9,    1,   94,    2, 0x06 /* Public */,
      10,    1,   97,    2, 0x06 /* Public */,
      12,    1,  100,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    1,  103,    2, 0x08 /* Private */,
      14,    0,  106,    2, 0x08 /* Private */,
      15,    1,  107,    2, 0x08 /* Private */,
      16,    0,  110,    2, 0x08 /* Private */,
      17,    1,  111,    2, 0x08 /* Private */,
      19,    0,  114,    2, 0x08 /* Private */,
      20,    0,  115,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void, 0x80000000 | 3,   11,
    QMetaType::Void, 0x80000000 | 3,   11,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   18,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->modelLoaded((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 1: _t->stateCheckBoxAxesChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->stateCheckBoxMeshSurfaceChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->buttonResetCameraClicked(); break;
        case 4: _t->stateCheckBoxShowVerticesChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->actionSaveModelTriggered((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 6: _t->actionSaveSegmentTriggered((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 7: _t->on_checkBoxAxis_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_actionOpenFile_triggered(); break;
        case 9: _t->on_checkBoxMeshSurface_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->on_pushButtonResetCamera_clicked(); break;
        case 11: _t->on_checkBoxShowVertices_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_actionSave_Model_triggered(); break;
        case 13: _t->on_actionSave_Segment_triggered(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MainWindow::*_t)(std::string );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::modelLoaded)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::stateCheckBoxAxesChanged)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::stateCheckBoxMeshSurfaceChanged)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::buttonResetCameraClicked)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::stateCheckBoxShowVerticesChanged)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(std::string );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::actionSaveModelTriggered)) {
                *result = 5;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(std::string );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::actionSaveSegmentTriggered)) {
                *result = 6;
                return;
            }
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::modelLoaded(std::string _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainWindow::stateCheckBoxAxesChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MainWindow::stateCheckBoxMeshSurfaceChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MainWindow::buttonResetCameraClicked()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void MainWindow::stateCheckBoxShowVerticesChanged(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void MainWindow::actionSaveModelTriggered(std::string _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void MainWindow::actionSaveSegmentTriggered(std::string _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_END_MOC_NAMESPACE
