/****************************************************************************
** Copyright (c) 2000-2003 Wayne Roth
** Copyright (c) 2004-2007 Stefan Sander
** Copyright (c) 2007 Michal Policht
** Copyright (c) 2008 Brandon Fosdick
** Copyright (c) 2009-2010 Liam Staskawicz
** Copyright (c) 2011 Debao Zhang
** All right reserved.
** Web: http://code.google.com/p/qextserialport/
**
** Permission is hereby granted, free of charge, to any person obtaining
** a copy of this software and associated documentation files (the
** "Software"), to deal in the Software without restriction, including
** without limitation the rights to use, copy, modify, merge, publish,
** distribute, sublicense, and/or sell copies of the Software, and to
** permit persons to whom the Software is furnished to do so, subject to
** the following conditions:
**
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
** NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
** LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
** OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
** WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
****************************************************************************/

#include "qextserialenumerator.h"
#include "qextserialenumerator_p.h"
#include <QtCore/QDebug>
#include <QtCore/QMetaType>
#include <QtCore/QRegExp>
#include <algorithm>
#include <objbase.h>
#include <initguid.h>
#include <setupapi.h>
#include <dbt.h>

#ifdef QT_GUI_LIB
/*!
  \internal
  \class QextSerialRegistrationWidget

  Internal window which is used to receive device arrvial and removal message.
*/

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#include <QtGui/QWidget>
class QextSerialRegistrationWidget : public QWidget
#else
#include <QtGui/QWindow>
class QextSerialRegistrationWidget : public QWindow
#endif
{
public:
    QextSerialRegistrationWidget(QextSerialEnumeratorPrivate *qese) {
        this->qese = qese;
    }
    ~QextSerialRegistrationWidget() {}

protected:

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    bool winEvent(MSG *message, long *result) {
#else
    bool nativeEvent(const QByteArray & /*eventType*/, void *msg, long *result) {
        MSG *message = static_cast<MSG *>(msg);
#endif
        if (message->message == WM_DEVICECHANGE) {
            qese->onDeviceChanged(message->wParam, message->lParam);
            *result = 1;
            return true;
        }
        return false;
    }
private:
    QextSerialEnumeratorPrivate *qese;
};
#endif // QT_GUI_LIB

void QextSerialEnumeratorPrivate::init_sys()
{
#ifdef QT_GUI_LIB
    notificationWidget = 0;
#endif // QT_GUI_LIB
}

/*!
  default
*/
void QextSerialEnumeratorPrivate::destroy_sys()
{
#ifdef QT_GUI_LIB
    if (notificationWidget)
        delete notificationWidget;
#endif
}

#ifndef GUID_DEVINTERFACE_COMPORT
DEFINE_GUID(GUID_DEVINTERFACE_COMPORT, 0x86e0d1e0L, 0x8089, 0x11d0, 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73);
#endif


/*!
    \internal
    Get value of specified property from the registry.
        \a key handle to an open key.
        \a property property name.

        return property value.
*/
static QString getRegKeyValue(HKEY key, LPCTSTR property)
{
    DWORD size = 0;
    DWORD type;
    if (::RegQueryValueEx(key, property, NULL, NULL, NULL, &size) != ERROR_SUCCESS)
        return QString();
    BYTE *buff = new BYTE[size];
    QString result;
    if (::RegQueryValueEx(key, property, NULL, &type, buff, &size) == ERROR_SUCCESS)
        result = QString::fromUtf16(reinterpret_cast<ushort *>(buff));
    delete [] buff;
    return result;
}

/*!
     \internal
     Get specific property from registry.
     \a devInfoSet pointer to the device information set that contains the interface
        and its underlying device. Returned by SetupDiGetClassDevs() function.
     \a devInfoData pointer to an SP_DEVINFO_DATA structure that defines the device instance.
        this is returned by SetupDiGetDeviceInterfaceDetail() function.
     \a property registry property. One of defined SPDRP_* constants.

     return property string.
 */
static QString getDeviceRegistryProperty(HDEVINFO devInfoSet, PSP_DEVINFO_DATA devInfoData, DWORD property)
{
    DWORD buffSize = 0;
    ::SetupDiGetDeviceRegistryProperty(devInfoSet, devInfoData, property, NULL, NULL, 0, &buffSize);
    if (GetLastError() != ERROR_INSUFFICIENT_BUFFER)
        return QString();
    BYTE *buff = new BYTE[buffSize];
    ::SetupDiGetDeviceRegistryProperty(devInfoSet, devInfoData, property, NULL, buff, buffSize, NULL);
    QString result = QString::fromUtf16(reinterpret_cast<ushort *>(buff));
    delete [] buff;
    return result;
}

/*!
     \internal
*/
static bool getDeviceDetailsInformation(QextPortInfo *portInfo, HDEVINFO devInfoSet, PSP_DEVINFO_DATA devInfoData
                                 , WPARAM wParam = DBT_DEVICEARRIVAL)
{
    portInfo->friendName = getDeviceRegistryProperty(devInfoSet, devInfoData, SPDRP_FRIENDLYNAME);
    if (wParam == DBT_DEVICEARRIVAL)
        portInfo->physName = getDeviceRegistryProperty(devInfoSet, devInfoData, SPDRP_PHYSICAL_DEVICE_OBJECT_NAME);
    portInfo->enumName = getDeviceRegistryProperty(devInfoSet, devInfoData, SPDRP_ENUMERATOR_NAME);

    HKEY devKey = ::SetupDiOpenDevRegKey(devInfoSet, devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
    portInfo->portName = getRegKeyValue(devKey, TEXT("PortName"));
    ::RegCloseKey(devKey);

    QString hardwareIDs = getDeviceRegistryProperty(devInfoSet, devInfoData, SPDRP_HARDWAREID);
    QRegExp idRx(QLatin1String("VID_(\\w+)&PID_(\\w+)"));
    if (hardwareIDs.toUpper().contains(idRx)) {
        bool dummy;
        portInfo->vendorID = idRx.cap(1).toInt(&dummy, 16);
        portInfo->productID = idRx.cap(2).toInt(&dummy, 16);
        //qDebug() << "got vid:" << vid << "pid:" << pid;
    }
    return true;
}

/*!
     \internal
*/
static void enumerateDevices(const GUID &guid, QList<QextPortInfo> *infoList)
{
    HDEVINFO devInfoSet = ::SetupDiGetClassDevs(&guid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (devInfoSet != INVALID_HANDLE_VALUE) {
        SP_DEVINFO_DATA devInfoData;
        devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
        for(int i = 0; ::SetupDiEnumDeviceInfo(devInfoSet, i, &devInfoData); i++) {
            QextPortInfo info;
            info.productID = info.vendorID = 0;
            getDeviceDetailsInformation(&info, devInfoSet, &devInfoData);
            infoList->append(info);
        }
        ::SetupDiDestroyDeviceInfoList(devInfoSet);
    }
}


static bool lessThan(const QextPortInfo &s1, const QextPortInfo &s2)
{
    if (s1.portName.startsWith(QLatin1String("COM"))
            && s2.portName.startsWith(QLatin1String("COM"))) {
        return s1.portName.mid(3).toInt()<s2.portName.mid(3).toInt();
    }
    return s1.portName < s2.portName;
}


/*!
    Get list of ports.

    return list of ports currently available in the system.
*/
QList<QextPortInfo> QextSerialEnumeratorPrivate::getPorts_sys()
{
    QList<QextPortInfo> ports;
    enumerateDevices(GUID_DEVINTERFACE_COMPORT, &ports);
    std::sort(ports.begin(), ports.end(), lessThan);
    return ports;
}


/*
    Enable event-driven notifications of board discovery/removal.
*/
bool QextSerialEnumeratorPrivate::setUpNotifications_sys(bool setup)
{
#ifndef QT_GUI_LIB
    Q_UNUSED(setup)
    QESP_WARNING("QextSerialEnumerator: GUI not enabled - can't register for device notifications.");
    return false;
#else
    Q_Q(QextSerialEnumerator);
    if (setup && notificationWidget) //already setup
        return true;
    notificationWidget = new QextSerialRegistrationWidget(this);

    DEV_BROADCAST_DEVICEINTERFACE dbh;
    ::ZeroMemory(&dbh, sizeof(dbh));
    dbh.dbcc_size = sizeof(dbh);
    dbh.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    // dbh.dbcc_classguid = GUID_DEVCLASS_PORTS; //Ignored in such case
    DWORD flags = DEVICE_NOTIFY_WINDOW_HANDLE|DEVICE_NOTIFY_ALL_INTERFACE_CLASSES;
    if (::RegisterDeviceNotification((HWND)notificationWidget->winId(), &dbh, flags) == NULL) {
        QESP_WARNING() << "RegisterDeviceNotification failed:" << GetLastError();
        return false;
    }
    // setting up notifications doesn't tell us about devices already connected
    // so get those manually
    foreach (QextPortInfo port, getPorts_sys())
        Q_EMIT q->deviceDiscovered(port);
    return true;
#endif // QT_GUI_LIB
}

#ifdef QT_GUI_LIB
LRESULT QextSerialEnumeratorPrivate::onDeviceChanged(WPARAM wParam, LPARAM lParam)
{
    if (DBT_DEVICEARRIVAL == wParam || DBT_DEVICEREMOVECOMPLETE == wParam) {
        PDEV_BROADCAST_HDR pHdr = (PDEV_BROADCAST_HDR)lParam;
        if (pHdr->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE) {
            PDEV_BROADCAST_DEVICEINTERFACE pDevInf = (PDEV_BROADCAST_DEVICEINTERFACE)pHdr;
             // delimiters are different across APIs...change to backslash.  ugh.
            QString deviceID = QString::fromUtf16(reinterpret_cast<ushort *>(pDevInf->dbcc_name));
            deviceID = deviceID.toUpper().replace(QLatin1String("#"), QLatin1String("\\"));

            matchAndDispatchChangedDevice(deviceID, GUID_DEVINTERFACE_COMPORT, wParam);
        }
    }
    return 0;
}

bool QextSerialEnumeratorPrivate::matchAndDispatchChangedDevice(const QString &deviceID, const GUID &guid, WPARAM wParam)
{
    Q_Q(QextSerialEnumerator);
    bool rv = false;
    DWORD dwFlag = (DBT_DEVICEARRIVAL == wParam) ? DIGCF_PRESENT : DIGCF_PROFILE;
    HDEVINFO devInfoSet  = SetupDiGetClassDevs(&guid, NULL, NULL, dwFlag | DIGCF_DEVICEINTERFACE);
    if (devInfoSet != INVALID_HANDLE_VALUE) {
        SP_DEVINFO_DATA spDevInfoData;
        spDevInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
        for(int i=0; SetupDiEnumDeviceInfo(devInfoSet, i, &spDevInfoData); i++) {
            DWORD nSize = 0;
            TCHAR buf[MAX_PATH];
            if (SetupDiGetDeviceInstanceId(devInfoSet, &spDevInfoData, buf, MAX_PATH, &nSize)
                    && deviceID.contains(QString::fromUtf16(reinterpret_cast<ushort *>(buf)))) { // we found a match
                rv = true;
                QextPortInfo info;
                info.productID = info.vendorID = 0;
                getDeviceDetailsInformation(&info, devInfoSet, &spDevInfoData, wParam);
                if (wParam == DBT_DEVICEARRIVAL)
                    Q_EMIT q->deviceDiscovered(info);
                else if (wParam == DBT_DEVICEREMOVECOMPLETE)
                    Q_EMIT q->deviceRemoved(info);
                break;
            }
        }
        SetupDiDestroyDeviceInfoList(devInfoSet);
    }
    return rv;
}
#endif //QT_GUI_LIB
