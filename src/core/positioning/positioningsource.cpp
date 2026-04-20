/***************************************************************************
  positioningsource.cpp - PositioningSource

 ---------------------
 begin                : 20.12.2024
 copyright            : (C) 2024 by Mathieu Pellerin
 email                : mathieu at opengis dot ch
 ***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef WITH_BLUETOOTH
#include "bluetoothreceiver.h"
#endif
#ifdef WITH_SERIALPORT
#include "serialportreceiver.h"
#endif
#include "egenioussreceiver.h"
#include "filereceiver.h"
#include "internalgnssreceiver.h"
#include "positioningsource.h"
#include "positioningutils.h"
#include "tcpreceiver.h"
#include "udpreceiver.h"

#include <cmath>
#include <QStandardPaths>

QString PositioningSource::backgroundFilePath = QStringLiteral( "%1/positioning.background" ).arg( QStandardPaths::writableLocation( QStandardPaths::AppDataLocation ) );

namespace
{
double wrapDegrees360( double angle )
{
  if ( std::isnan( angle ) )
    return angle;

  double wrapped = std::fmod( angle, 360.0 );
  if ( wrapped < 0.0 )
    wrapped += 360.0;

  return wrapped;
}
}

PositioningSource::PositioningSource( QObject *parent )
  : QObject( parent )
{
  // Setup internal gnss receiver by default
  setupDevice();

  // Setup the compass, use a timer instead of the compass's readingChanged signal to avoid handling
  // too many signals
  mCompassTimer.setInterval( 200 );
  connect( &mCompassTimer, &QTimer::timeout, this, &PositioningSource::processCompassReading );

  // Poll internal attitude sensors at the same cadence to populate IMU fields when using the device's sensors.
  mAttitudeTimer.setInterval( 200 );
  connect( &mAttitudeTimer, &QTimer::timeout, this, &PositioningSource::processRotationReading );
  connect( &mAttitudeTimer, &QTimer::timeout, this, &PositioningSource::processTiltReading );
}

void PositioningSource::setActive( bool active )
{
  if ( mActive == active )
    return;

  mActive = active;

  if ( mActive )
  {
    if ( !mReceiver )
    {
      setupDevice();
    }
    else
    {
      mReceiver->connectDevice();
    }
    if ( !QSensor::sensorsForType( QCompass::sensorType ).isEmpty() )
    {
      mCompass.setActive( true );
      mCompassTimer.start();
    }
    if ( mDeviceId.isEmpty() )
    {
      if ( !QSensor::sensorsForType( QRotationSensor::sensorType ).isEmpty() )
      {
        mRotationSensor.setActive( true );
      }
      if ( !QSensor::sensorsForType( QTiltSensor::sensorType ).isEmpty() )
      {
        mTiltSensor.setActive( true );
      }
      if ( mRotationSensor.isActive() || mTiltSensor.isActive() )
      {
        mAttitudeTimer.start();
      }
    }
  }
  else
  {
    if ( mReceiver )
    {
      mReceiver->disconnectDevice();
    }
    mCompassTimer.stop();
    mCompass.setActive( false );
    mAttitudeTimer.stop();
    mRotationSensor.setActive( false );
    mTiltSensor.setActive( false );
    mOrientation = std::numeric_limits<double>::quiet_NaN();
    mInternalImuRoll = std::numeric_limits<double>::quiet_NaN();
    mInternalImuPitch = std::numeric_limits<double>::quiet_NaN();
    mInternalImuHeading = std::numeric_limits<double>::quiet_NaN();
    mInternalImuSteering = std::numeric_limits<double>::quiet_NaN();
    emit orientationChanged();
  }

  emit activeChanged();
}

void PositioningSource::setDeviceId( const QString &id )
{
  if ( mDeviceId == id )
    return;

  mDeviceId = id;
  setupDevice();

  emit deviceIdChanged();
}

void PositioningSource::setValid( bool valid )
{
  if ( mValid == valid )
    return;

  mValid = valid;

  emit validChanged();
}

void PositioningSource::setLogging( bool logging )
{
  if ( mLogging == logging )
    return;

  mLogging = logging;

  if ( mReceiver )
  {
    if ( mLogging && !mLoggingPath.isEmpty() )
    {
      mReceiver->startLogging( mLoggingPath );
    }
    else
    {
      mReceiver->stopLogging();
    }
  }

  emit loggingChanged();
}

void PositioningSource::setLoggingPath( const QString &path )
{
  if ( mLoggingPath == path )
    return;

  mLoggingPath = path;

  if ( mReceiver && mLogging )
  {
    mReceiver->startLogging( mLoggingPath );
  }

  emit loggingPathChanged();
}

void PositioningSource::setBackgroundMode( bool backgroundMode )
{
  if ( mBackgroundMode == backgroundMode )
    return;

  mBackgroundMode = backgroundMode;

  if ( mBackgroundMode )
  {
    if ( QFile::exists( QStringLiteral( "%1.information" ).arg( backgroundFilePath ) ) )
    {
      // Remove previously collected position information
      QFile::remove( QStringLiteral( "%1.information" ).arg( backgroundFilePath ) );
    }
  }

  emit backgroundModeChanged();
}

QList<GnssPositionInformation> PositioningSource::getBackgroundPositionInformation() const
{
  QList<GnssPositionInformation> positionInformationList;

  QFile file( QStringLiteral( "%1.information" ).arg( backgroundFilePath ) );
  if ( file.exists() )
  {
    if ( file.open( QFile::ReadOnly ) )
    {
      QDataStream stream( &file );
      while ( !stream.atEnd() )
      {
        GnssPositionInformation positionInformation;
        stream >> positionInformation;
        positionInformationList << positionInformation;
      }
      file.close();
    }
  }

  return positionInformationList;
}

void PositioningSource::setElevationCorrectionMode( ElevationCorrectionMode elevationCorrectionMode )
{
  if ( mElevationCorrectionMode == elevationCorrectionMode )
    return;

  mElevationCorrectionMode = elevationCorrectionMode;

  emit elevationCorrectionModeChanged();
}

void PositioningSource::setAntennaHeight( double antennaHeight )
{
  if ( mAntennaHeight == antennaHeight )
    return;

  mAntennaHeight = antennaHeight;

  emit antennaHeightChanged();
}

void PositioningSource::setupDevice()
{
  mInternalImuRoll = std::numeric_limits<double>::quiet_NaN();
  mInternalImuPitch = std::numeric_limits<double>::quiet_NaN();
  mInternalImuHeading = std::numeric_limits<double>::quiet_NaN();
  mInternalImuSteering = std::numeric_limits<double>::quiet_NaN();

  if ( mReceiver )
  {
    mReceiver->disconnectDevice();
    mReceiver->stopLogging();
    disconnect( mReceiver.get(), &AbstractGnssReceiver::lastGnssPositionInformationChanged, this, &PositioningSource::lastGnssPositionInformationChanged );
    disconnect( mReceiver.get(), &AbstractGnssReceiver::lastErrorChanged, this, &PositioningSource::deviceLastErrorChanged );
    disconnect( mReceiver.get(), &AbstractGnssReceiver::socketStateChanged, this, &PositioningSource::deviceSocketStateChanged );
    disconnect( mReceiver.get(), &AbstractGnssReceiver::socketStateStringChanged, this, &PositioningSource::deviceSocketStateStringChanged );
    mReceiver->deleteLater();
    mReceiver.reset();
  }

  if ( mDeviceId.isEmpty() )
  {
    mReceiver = std::make_unique<InternalGnssReceiver>( this );
  }
  else
  {
    if ( mDeviceId.startsWith( FileReceiver::identifier + ":" ) )
    {
      const qsizetype prefixLength = FileReceiver::identifier.length() + 1;
      const qsizetype intervalSeparator = mDeviceId.lastIndexOf( ':' );
      const QString filePath = mDeviceId.mid( prefixLength, intervalSeparator - prefixLength );
      const int interval = mDeviceId.mid( intervalSeparator + 1 ).toInt();
      mReceiver = std::make_unique<FileReceiver>( filePath, interval, this );
    }
    else if ( mDeviceId.startsWith( TcpReceiver::identifier + ":" ) )
    {
      const qsizetype prefixLength = TcpReceiver::identifier.length() + 1;
      const qsizetype portSeparator = mDeviceId.lastIndexOf( ':' );
      const QString address = mDeviceId.mid( prefixLength, portSeparator - prefixLength );
      const int port = mDeviceId.mid( portSeparator + 1 ).toInt();
      mReceiver = std::make_unique<TcpReceiver>( address, port, this );
    }
    else if ( mDeviceId.startsWith( UdpReceiver::identifier + ":" ) )
    {
      const qsizetype prefixLength = UdpReceiver::identifier.length() + 1;
      const qsizetype portSeparator = mDeviceId.lastIndexOf( ':' );
      const QString address = mDeviceId.mid( prefixLength, portSeparator - prefixLength );
      const int port = mDeviceId.mid( portSeparator + 1 ).toInt();
      mReceiver = std::make_unique<UdpReceiver>( address, port, this );
    }
    else if ( mDeviceId.startsWith( EgenioussReceiver::identifier + ":" ) )
    {
      const qsizetype prefixLength = EgenioussReceiver::identifier.length() + 1;
      const qsizetype portSeparator = mDeviceId.lastIndexOf( ':' );
      const QString address = mDeviceId.mid( prefixLength, portSeparator - prefixLength );
      const int port = mDeviceId.mid( portSeparator + 1 ).toInt();
      mReceiver = std::make_unique<EgenioussReceiver>( address, port, this );
    }
#ifdef WITH_SERIALPORT
    else if ( mDeviceId.startsWith( SerialPortReceiver::identifier + ":" ) )
    {
      const qsizetype prefixLength = SerialPortReceiver::identifier.length() + 1;
      const QString address = mDeviceId.mid( prefixLength );
      mReceiver = std::make_unique<SerialPortReceiver>( address, this );
    }
#endif
    else
    {
#ifdef WITH_BLUETOOTH
      mReceiver = std::make_unique<BluetoothReceiver>( mDeviceId, this );
#endif
    }
  }

  // Reset the position information to insure no cross contamination between receiver types
  lastGnssPositionInformationChanged( GnssPositionInformation() );
  connect( mReceiver.get(), &AbstractGnssReceiver::lastGnssPositionInformationChanged, this, &PositioningSource::lastGnssPositionInformationChanged );
  connect( mReceiver.get(), &AbstractGnssReceiver::lastErrorChanged, this, &PositioningSource::deviceLastErrorChanged );
  connect( mReceiver.get(), &AbstractGnssReceiver::socketStateChanged, this, &PositioningSource::deviceSocketStateChanged );
  connect( mReceiver.get(), &AbstractGnssReceiver::socketStateStringChanged, this, &PositioningSource::deviceSocketStateStringChanged );
  setValid( mReceiver->valid() );

  emit deviceChanged();

  if ( mLogging && !mLoggingPath.isEmpty() )
  {
    mReceiver->startLogging( mLoggingPath );
  }

  if ( mActive )
  {
    mReceiver->connectDevice();
  }

  return;
}

void PositioningSource::lastGnssPositionInformationChanged( const GnssPositionInformation &lastGnssPositionInformation )
{
  const bool useInternalImu = mDeviceId.isEmpty();
  const double imuRoll = lastGnssPositionInformation.imuRollValid() ? lastGnssPositionInformation.imuRoll() : ( useInternalImu ? mInternalImuRoll : std::numeric_limits<double>::quiet_NaN() );
  const double imuPitch = lastGnssPositionInformation.imuPitchValid() ? lastGnssPositionInformation.imuPitch() : ( useInternalImu ? mInternalImuPitch : std::numeric_limits<double>::quiet_NaN() );
  const double imuHeading = lastGnssPositionInformation.imuHeadingValid() ? lastGnssPositionInformation.imuHeading() : ( useInternalImu ? mInternalImuHeading : std::numeric_limits<double>::quiet_NaN() );
  const double imuSteering = lastGnssPositionInformation.imuSteeringValid() ? lastGnssPositionInformation.imuSteering() : ( useInternalImu ? mInternalImuSteering : std::numeric_limits<double>::quiet_NaN() );
  const bool imuCorrection = lastGnssPositionInformation.imuCorrection() || ( useInternalImu && ( !std::isnan( imuRoll ) || !std::isnan( imuPitch ) || !std::isnan( imuHeading ) || !std::isnan( imuSteering ) ) );

  const GnssPositionInformation positionInformation( lastGnssPositionInformation.latitude(),
                                                     lastGnssPositionInformation.longitude(),
                                                     lastGnssPositionInformation.elevation(),
                                                     lastGnssPositionInformation.speed(),
                                                     lastGnssPositionInformation.direction(),
                                                     lastGnssPositionInformation.satellitesInView(),
                                                     lastGnssPositionInformation.pdop(),
                                                     lastGnssPositionInformation.hdop(),
                                                     lastGnssPositionInformation.vdop(),
                                                     lastGnssPositionInformation.hacc(),
                                                     lastGnssPositionInformation.vacc(),
                                                     lastGnssPositionInformation.utcDateTime().isValid() ? lastGnssPositionInformation.utcDateTime() : QDateTime::currentDateTimeUtc(),
                                                     lastGnssPositionInformation.fixMode(),
                                                     lastGnssPositionInformation.fixType(),
                                                     lastGnssPositionInformation.quality(),
                                                     lastGnssPositionInformation.satellitesUsed(),
                                                     lastGnssPositionInformation.status(),
                                                     lastGnssPositionInformation.satPrn(),
                                                     lastGnssPositionInformation.satInfoComplete(),
                                                     lastGnssPositionInformation.verticalSpeed(),
                                                     lastGnssPositionInformation.magneticVariation(),
                                                     lastGnssPositionInformation.averagedCount(),
                                                     lastGnssPositionInformation.sourceName(),
                                                     imuCorrection,
                                                     imuRoll,
                                                     imuPitch,
                                                     imuHeading,
                                                     imuSteering,
                                                     mOrientation );

  if ( mPositionInformation == positionInformation )
    return;

  mPositionInformation = positionInformation;

  if ( !mBackgroundMode )
  {
    emit positionInformationChanged();
  }
  else
  {
    QFile file( QStringLiteral( "%1.information" ).arg( backgroundFilePath ) );
    if ( file.open( QFile::Append ) )
    {
      QDataStream stream( &file );
      stream << mPositionInformation;
      file.close();
    }
  }
}

void PositioningSource::processCompassReading()
{
  if ( mCompass.reading() )
  {
    double orientation = 0.0;
    orientation += mCompass.reading()->azimuth();
    if ( orientation < 0.0 )
    {
      orientation = 360 + orientation;
    }

    if ( mOrientation != orientation )
    {
      mOrientation = orientation;
      if ( !mBackgroundMode )
      {
        emit orientationChanged();
      }
    }
  }
}

void PositioningSource::processRotationReading()
{
  if ( !mRotationSensor.isActive() || !mRotationSensor.reading() )
    return;

  const double roll = mRotationSensor.reading()->y();
  const double pitch = mRotationSensor.reading()->x();
  const double heading = wrapDegrees360( mRotationSensor.reading()->z() );
  const double steering = !std::isnan( mOrientation ) ? mOrientation : heading;

  if ( !qgsDoubleNear( mInternalImuRoll, roll ) )
    mInternalImuRoll = roll;
  if ( !qgsDoubleNear( mInternalImuPitch, pitch ) )
    mInternalImuPitch = pitch;
  if ( !qgsDoubleNear( mInternalImuHeading, heading ) )
    mInternalImuHeading = heading;
  if ( !qgsDoubleNear( mInternalImuSteering, steering ) )
    mInternalImuSteering = steering;

  if ( mDeviceId.isEmpty() && mPositionInformation.isValid() )
  {
    lastGnssPositionInformationChanged( mReceiver ? mReceiver->lastGnssPositionInformation() : GnssPositionInformation() );
  }
}

void PositioningSource::processTiltReading()
{
  if ( !mTiltSensor.isActive() || !mTiltSensor.reading() )
    return;

  // Use QTiltSensor only as a fallback when QRotationSensor values are unavailable.
  if ( std::isnan( mInternalImuPitch ) )
    mInternalImuPitch = mTiltSensor.reading()->xRotation();
  if ( std::isnan( mInternalImuRoll ) )
    mInternalImuRoll = mTiltSensor.reading()->yRotation();

  if ( mDeviceId.isEmpty() && mPositionInformation.isValid() )
  {
    lastGnssPositionInformationChanged( mReceiver ? mReceiver->lastGnssPositionInformation() : GnssPositionInformation() );
  }
}

void PositioningSource::triggerConnectDevice()
{
  if ( mReceiver )
  {
    mReceiver->connectDevice();
  }
}

void PositioningSource::triggerDisconnectDevice()
{
  if ( mReceiver )
  {
    mReceiver->disconnectDevice();
  }
}
