import serial
import io
import re
import time
import json
import logging
import atexit
import sys
import threading
from PyQt5 import QtGui, QtCore,  QtWidgets

# import threading
# from   select    import select


log = logging.getLogger(__name__)

#####################################################################
# Error messages, taken from other project
#####################################################################

class PumpError(Exception):
    '''
    General pump error
    '''

    def __init__(self, code, mesg=None):
        self.code = code
        self.mesg = mesg

    def __str__(self):
        result = '%s\n\n%s' % (self._todo, self._mesg[self.code])
        if self.mesg is not None:
            result += ' ' + self.mesg
        return result

class PumpCommError(PumpError):
    '''
    Handles error messages resulting from problems with communication via the
    pump's serial port.
    '''

    _mesg = {
            # Actual codes returned by the pump
            ''      : 'Command is not recognized',
            'NA'    : 'Command is not currently applicable',
            'OOR'   : 'Command data is out of range',
            'COM'   : 'Invalid communications packet received',
            'IGN'   : 'Command ignored due to new phase start',
            # Custom codes
            'NR'    : 'No response from pump',
            'SER'   : 'Unable to open serial port',
            'UNK'   : 'Unknown error',
            }

    _todo = 'Unable to connect to pump.  Please ensure that no other ' + \
            'programs that utilize the pump are running and try ' + \
            'try power-cycling the entire system (rack and computer).'

class PumpHardwareError(PumpError):

    '''Handles errors specific to the pump hardware and firmware.'''

    _mesg = {
            'R'     : 'Pump was reset due to power interrupt',
            'S'     : 'Pump motor is stalled',
            'T'     : 'Safe mode communication time out',
            'E'     : 'Pumping program error',
            'O'     : 'Pumping program phase out of range',
            }

    _todo = 'Pump has reported an error.  Please check to ensure pump ' + \
            'motor is not over-extended and power-cycle the pump.'

class PumpUnitError(Exception):
    '''Occurs when the pump returns a value in an unexpected unit
    '''

    def __init__(self, expected, actual, cmd):
        self.expected = expected
        self.actual = actual
        self.cmd = cmd

    def __str__(self):
        mesg = '%s: Expected units in %s, receved %s'
        return mesg % (self.cmd, self.expected, self.actual)

#####################################################################
# Pump Class
#####################################################################

class Pump(object):
    '''
    Initiating a pump is done with a dictionary that contains any of 'rate,'diameter','rate-units', and 'address'. If any is missing, a default value is assigned. 
    For now I'm assuming only infusion.
    '''

    ETX = '\x03'    # End of packet transmission
    STX = '\x02'    # Start of packet transmission
    CR  = '\x0D'    # Carriage return

    DEFAULTS = dict(baudrate=19200, bytesize=8, parity='N',
            stopbits=1, timeout=.1, xonxoff=0, rtscts=0, writeTimeout=1,
            dsrdtr=None, interCharTimeout=None)

    STATUS = dict(I='infusing', W='withdrawing', S='halted', P='paused',
                  T='in timed pause', U='waiting for trigger', X='purging')

    RATE_UNIT = {'UM':'ul/min','MM':'ml/min','UH':'ul/h','MH':'ml/h'}
    VOL_UNIT = {'UL':'ul','ML':'ml'}

    DIR_MODE = {
        'INF': 'infuse',
        'WDR': 'withdraw',
        'REV': 'reverse',
    }

    mmsg=''

    REV_DIR_MODE = dict((v, k) for k, v in DIR_MODE.items())

    _response = re.compile('\x02' + '(?P<address>\d+)' + \
                                 '(?P<status>[IWSPTUX]|A\?)' + \
                                 '(?P<data>.*)' + '\x03')
    _dispensed = re.compile('I(?P<infuse>[\.0-9]+)' + \
                            'W(?P<withdraw>[\.0-9]+)' + \
                            '(?P<units>[MLU]{2})')

    def __init__(self,ser,config):
        mmsg = ''
        self._ser = ser
        if 'rate' in config:
            self._rate = config['rate']
        else:
            self._rate = 10
        if 'diameter' in config:
            self._diameter = config['diameter']
        else:
            self._diameter = 14.3
        if 'rate-units' in config:
            self._rate_units = config['rate-units']
        else:
            self._rate_units = 'UM'
        if 'address' in config:
            self._address = config['address']
        else:
            self._address = 0
        if 'volume' in config:
            self._volume = config['volume']
        else:
            self._volume = 100
        if 'vol-units' in config:
            self._volume_unit = config['vol-units']
        else:
            self._volume_unit = 'UL'
        try:
            mmsg = self._write_read('AL 0')    # Turn audible alarm on. -- for some reason fails in some pumps.
            self._write_read('DIA ' + str(self._diameter))
            self._write_read('VOL ' + str(self._volume))
            self._write_read('VOL ' + self._volume_unit)
            self._write_read('RAT ' + str(self._rate) + ' ' + self._rate_units) #'UM') #
            print('Initialized pump {} {}'.format(self._address,mmsg))
            self._lastInfused = 0
            self._lastWithdrawn = 0
            self.resetDispensed()
            atexit.register(self.disconnect)

            #  ((self._read_check('DIA',_self.diameter),
            #         self._read_check('VOL',_self.volume),
            #         self._read_check('RAT',str(_self.rate)+self._rate_units)))
        except PumpHardwareError as e:
            # We want to trap and dispose of one very specific exception code,
            # 'R', which corresponds to a power interrupt.  This is almost
            # always returned when the pump is first powered on and initialized
            # so it really is not a concern to us.  The other error messages are
            # of concern so we reraise them.
            print('0 {} {}'.format(mmsg,self._address))
            if e.code != 'R':
                raise e
        except NameError as e:
            # Raised when it cannot find the global name 'SERIAL' (which
            # typically indicates a problem connecting to COM1).  Let's
            # translate this to a human-understandable error.
            print('1 {} {}'.format(mmsg,self._address))
            log.exception(e)
            raise PumpCommError('SER')

    def _write_read(self, cmd):
        cmd = str(self._address) + ' ' + cmd + '\r'
        self._ser.write(cmd.encode('utf-8'))
        res=self._ser.readline()
        print('***************',res)
        if res == '':
            raise PumpCommError('NR', cmd)
        match = self._response.match(res.decode('utf-8'))
        if match is None:
            raise PumpCommError('NR')
        if match.group('status') == 'A?':
            raise PumpHardwareError(match.group('data'), cmd)
        elif match.group('data').startswith('?'):
            raise PumpCommError(match.group('data')[1:], cmd)
        return match.groupdict()

    def _read_check(self,ser, adr, cmd, value):
        cmd = str(self._address) + ' ' + cmd + '\r'
        res = self._write_read(cmd)
        if cmd =='DIS':
            res=_dispensed.match(res['data'].decode('utf-8'))
            res = res.groupdict()
            return(res[self._direction]==str(value))
        return(res['data']==str(value))


    def disconnect(self):
        ''' Stop pump and close serial port. Automatically called when Python exits. '''
        try:
            if self.getStatus() != 'halted':
                self.stop()
        finally:
            if self._ser.is_open:
                self._ser.close()
            return # Don't reraise error conditions, just quit silently     b b
    def run(self):
        ''' Starts the pump. '''
        self._write_read('RUN')
    def stop(self):
        ''' Stop the pump. Raises PumpError if the pump is already stopped. '''
        self._write_read('STP')
    def setDirection(self, direction):
        ''' Set direction of the pump. Valid directions are 'infuse', 'withdraw' and 'reverse'. '''
        self._write_read('DIR {}'.format(self.REV_DIR_MODE[direction]))
    def sendCommand(self,cmd):
        '''send any command'''
        self._write_read(cmd)
    def getDirection(self):
        ''' Get current direction of the pump. Response will be either 'infuse' or 'withdraw'. '''
        return self.DIR_MODE[self._write_read('DIR')['data']]
    def getRate(self):
        ''' Get current rate of the pump. '''
        return self._write_read('RAT')['data']
        #self.DIR_MODE[self._write_read('RAT')['data']]
    def setRate(self, rate):
        ''' Set current rate of the pump. '''
        return self._write_read('RAT {} {}'.format(rate, self._rate_units))
    def setVolume(self, volume, unit=None):
        ''' Set current volume of the pump '''
        return self._write_read('VOL {}'.format(volume))

    '''not tested'''
    def getVolume(self, unit=None):
        ''' Get current volume of the pump '''
        if SIM: return
        return self._write_read('VOL')['data']
    def _getDispensed(self, direction):
        # Helper method for getInfused and getWithdrawn
        result = self._write_read('DIS')['data']
        #log.debug('_dispensed: %s, result: %s', self._dispensed, result)
        match = self._dispensed.match(result)
        # if match.group('units') != self.volume_unit_cmd:
        #     raise PumpUnitError('ML', match.group('units'), 'DIS')
        # else:
        return float(match.group(direction))
    def resetDispensed(self):
        ''' Reset dispense measures '''
        self._lastInfused   = 0
        self._lastWithdrawn = 0
        self._write_read('CLD INF')
        self._write_read('CLD WDR')
    def getInfused(self):
        ''' Get total volume withdrawn '''
        return self._getDispensed('infuse') + self._lastInfused
    def getWithdrawn(self):
        ''' Get total volume dispensed. '''
        return self._getDispensed('withdraw') + self._lastWithdrawn
    def setDiameter(self, diameter):
        ''' Set diameter (unit must be mm). '''
        self._lastInfused   += self._getDispensed('infuse'  )
        self._lastWithdrawn += self._getDispensed('withdraw')
        self._write_read('DIA {}'.format(diameter))
    def getDiameter(self):
        ''' Get diameter setting in mm. '''
        return self._write_read('DIA')
    def getAddress(self):
        ''' Get diameter setting in mm. '''
        return self._address
    # def getTTL(self):
    #     ''' Get status of TTL trigger. '''
    #     tr = self._write_read('IN 2')
    #     if tr in ['0','1']:
    #         data = bool(eval(tr))
    #     else:
    #         raise PumpCommError('', 'IN 2')

    # getStatus is tested
    def getStatus(self):
        return self.STATUS[self._write_read('')['status']]

#####################################################################
# GUI
#####################################################################

class PumpControl(QtWidgets.QWidget):

    syringes = {'1 ml BD':'4.699',
                '3 ml BD':'8.585',
                '10 ml BD':'14.60',
                '30ml BD':'21.59'}
   
    def __init__(self,ser,pumps,prog_dict):
        super(PumpControl, self).__init__()
        self._ser = ser
        self._pumps = pumps
        self._prog_dict = prog_dict
        self._update_status_time = 4
        self.initUI()
        
    def initUI(self):      
        
        # set grid layout
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(10)
        
        # setup two buttons along top
        self.runbtn = QtWidgets.QPushButton('does nothing',self)
        grid.addWidget(self.runbtn,1,2)
        self.runbtn.setCheckable(True)
        # self.runbtn.clicked.connect(self.run_update)

        self.stopbtn = QtWidgets.QPushButton('also nothing',self)
        grid.addWidget(self.stopbtn,1,3)
        self.stopbtn.setCheckable(False)
        # self.stopbtn.clicked.connect(self.stop_all)

        # optional column labels
        grid.addWidget(QtWidgets.QLabel('Pump number'),2,0)
        grid.addWidget(QtWidgets.QLabel('Program'),2,1)
        grid.addWidget(QtWidgets.QLabel('Contents'),2,2)
        grid.addWidget(QtWidgets.QLabel('Volume ul'),2,3)
        grid.addWidget(QtWidgets.QLabel('Flow rate ul/min'), 2, 4)
        grid.addWidget(QtWidgets.QLabel('Current flow rate ul/min'),2,5)
          
        # interate over pumps, adding a row for each
        self.mapper = QtCore.QSignalMapper(self)
        self.runmapper = QtCore.QSignalMapper(self)   # run program
        self.runmanmapper = QtCore.QSignalMapper(self) # run manual
        self.stopmapper = QtCore.QSignalMapper(self)
        self.currflow = []
        self.vol = []
        self.rates = []
        # self.prime_btns = dict()
        self.run_btns = []
        self.run_man_btns = []
        self.stop_btns = []

        self._prog = list(range(len(self._pumps)))


        for i,pump in enumerate(self._pumps):
            row = 3+i
            
            # add pump number
            pumplab = QtWidgets.QLabel('Pump #{}'.format(pump.getAddress()))
            pumplab.setAlignment(QtCore.Qt.AlignHCenter)
            grid.addWidget(pumplab,row,0)

            # # add syringe pulldown
            # combo = QtWidgets.QComboBox(self)
            # [combo.addItem(s) for s in sorted(PumpControl.syringes)]
            # self.mapper.setMapping(combo,i)
            # combo.activated.connect(self.mapper.map)
            # grid.addWidget(combo,row,1)

            # add programs pulldown
            combo = QtWidgets.QComboBox(self)
            for prog in self._prog_dict:
                combo.addItem(prog)
            self.mapper.setMapping(combo,i)
            combo.activated.connect(self.mapper.map)
            grid.addWidget(combo,row,1)
            self.set_program(i)

            # add textbox to put syring contents
            grid.addWidget(QtWidgets.QLineEdit(),row,2)

            self.vol.append(QtWidgets.QLineEdit(self))
            grid.addWidget(self.vol[i],row,3)


            # add textbox to enter flow rates
            self.rates.append(QtWidgets.QLineEdit(self))
            grid.addWidget(self.rates[i],row,4)

            # add label to show current flow rates
            self.currflow.append(QtWidgets.QLabel(self))
            self.currflow[i].setAlignment(QtCore.Qt.AlignHCenter)
            grid.addWidget(self.currflow[i],row,5)

            # add run button
            btn = QtWidgets.QPushButton('Run Prog',self)
            btn.setCheckable(True)# makes the button toggleable
            self.runmapper.setMapping(btn,i)
            btn.clicked.connect(self.runmapper.map)
            grid.addWidget(btn,row,6)
            self.run_btns.append(btn)

            # add run manual button
            btn = QtWidgets.QPushButton('Run Man',self)
            btn.setCheckable(True)
            self.runmanmapper.setMapping(btn,i)
            btn.clicked.connect(self.runmanmapper.map)
            grid.addWidget(btn,row,7)
            self.run_man_btns.append(btn)


            # add stop button
            btn = QtWidgets.QPushButton('Stop',self)
            btn.setCheckable(False)# makes the button toggleable
            self.stopmapper.setMapping(btn,i)
            btn.clicked.connect(self.stopmapper.map)
            grid.addWidget(btn,row,8)
            self.stop_btns.append(btn)


            # # add prime button
            # btn = QtWidgets.QPushButton('Prime',self)
            # btn.setCheckable(True)# makes the button toggleable
            # self.primemapper.setMapping(btn,i)
            # btn.clicked.connect(self.primemapper.map)
            # grid.addWidget(btn,row,8)
            # self.prime_btns[i] = btn


        # mapper thing
        self.mapper.mapped.connect(self.set_program)
        self.runmapper.mapped.connect(self.run_pump_prog)
        self.runmanmapper.mapped.connect(self.run_pump_manual)
        self.stopmapper.mapped.connect(self.stop_pump)
         # self.primemapper.mapped.connect(self.prime_pumps)

        # set up the status bar
        self.curr_state = 'add later'
        self.statusbar = QtWidgets.QLabel(self)
        grid.addWidget(self.statusbar,1,4)
        self.statusbar.setText('Status: '+self.curr_state)

        # set up the error bar
        self.error_state = 'None'
        self.errorbar = QtWidgets.QLabel(self)
        grid.addWidget(self.errorbar,1,5)
        self.errorbar.setText('Error: '+self.error_state)
        
        # set up the last command bar
        self.commandbar = QtWidgets.QLabel(self)
        grid.addWidget(self.commandbar,row+1,0,1,4)
        
        # make the prime state: a set containing the priming pumps
        self.prime_state = set()

        #initialize: set all flow rates to zero
        # self.run_update()
        # self.stop_all()
        # [self.update_syringe(p) for p in self._pumps]
        self.commandbar.setText('where is this')

        # keyboard shortcuts
        # QtWidgets.QShortcut(QtGui.QKeySequence('Space'),self,self.stop_all)

        # format the page
        self.setLayout(grid)
        self.setWindowTitle('Pump control')
        #self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint) # always on top
        self.show()

        serial_lock = threading.Lock()

        #  launch the thread here
        ## This is the end, __init__
        self.check_rates_loop()
        self.t = threading.Timer(self._update_status_time,self.check_rates_loop)
        self.t.start()

    def set_program(self, i):
        self._prog[i] = self.mapper.mapping(i).currentText()
        #self._prog_dict[]
        print('just change the program of pump {} to {}'.format(i,self._prog[i]))

    def set_vol(self,i): # must be reset the run command to take effect
        self._vol[i] = self.mappervol.mapping(i).currentText()

    def run_pump_prog(self, i):
        # temporary thing. this thing should actually run the program
        # still need to add check if volume written
        print('trying pump {} '.format(i))
        print(self.run_btns[i].isChecked())
        if self.run_btns[i].isChecked():
            if self.run_man_btns[i].isChecked():
                self.run_man_btns[i].setChecked(False)
                if self._pumps[i].getStatus() != 'halted':
                    self._pumps[i].stop()
                else:
                    print('pump halted but button is checked')
            print('I''m running pump {} '.format(i))
            # send seq of commands
            if self._prog[i] != 'pulse w/ w' and self._prog[i] != 'wash':
                print('got to 1')
                this_prog = self._prog_dict[self._prog[i]]
                print('got to 2')
                volploop = this_prog['pulse rate'] * this_prog['pulse duration'] + this_prog['flow rate'] * this_prog['pulse frequency']
                print('got to 3')
                '''check to see if there is enough volume for a loop'''
                if int(str(self.vol[i].text())) <= volploop*2:
                    self.run_btns[i].setChecked(False)
                    self.error_state = 'pump {}: not enough vol for prog'.format(i)
                    self.errorbar.setText('Error: ' + self.error_state)
                    print('got to 3.5')
                else:
                    loops = round(int(str(self.vol[i].text()))/volploop) - 1
                    print('got to 4')
                    self._pumps[i].sendCommand('PHN  1')
                    self._pumps[i].sendCommand('FUN RAT')
                    self._pumps[i].sendCommand('RAT {} {}'.format(this_prog['pulse rate'],'UM'))
                    self._pumps[i].sendCommand('VOL {}'.format(this_prog['pulse rate'] * this_prog['pulse duration']))
                    self._pumps[i].sendCommand('DIR INF')
                #('phase 2')

                    self._pumps[i].sendCommand('PHN  2')
                    self._pumps[i].sendCommand('FUN RAT')
                    self._pumps[i].sendCommand('RAT {} {}'.format(this_prog['flow rate'],'UM'))
                    self._pumps[i].sendCommand('VOL {}'.format(this_prog['flow rate'] * this_prog['pulse frequency']))
                    self._pumps[i].sendCommand('DIR INF')
                #print('phase 3')

                    self._pumps[i].sendCommand('PHN  3')
                    self._pumps[i].sendCommand('FUN LOP {}'.format(loops))
                #print('phase 4')

                    self._pumps[i].sendCommand('PHN  4')
                    self._pumps[i].sendCommand('FUN STP')
                    self._pumps[i].run()

            elif self._prog[i] == 'pulse w/ w': ## not complete
                pasT = 5

                self._pumps[i].sendCommand('PHN  1')
                self._pumps[i].sendCommand('FUN LPS')

                self._pumps[i].sendCommand('PHN  2')
                self._pumps[i].sendCommand('FUN PAS 60')

                self._pumps[i].sendCommand('PHN  3')
                self._pumps[i].sendCommand('FUN LOP 5') # loop the 60s pause 5 times

                self._pumps[i].sendCommand('PHN  4') # pulse flow 1
                self._pumps[i].sendCommand('FUN RAT')
                self._pumps[i].sendCommand('RAT {} {}'.format(str(100), 'UM'))
                self._pumps[i].sendCommand('VOL {}'.format(str(34)))
                self._pumps[i].sendCommand('DIR INF')

                self._pumps[i].sendCommand('PHN  5') # reg flow1
                self._pumps[i].sendCommand('FUN RAT')
                self._pumps[i].sendCommand('RAT {} {}'.format(str(3), 'UM'))
                self._pumps[i].sendCommand('VOL {}'.format(str(59)))
                self._pumps[i].sendCommand('DIR INF')

                self._pumps[i].sendCommand('PHN  6')  # pulse flow 2
                self._pumps[i].sendCommand('FUN RAT')
                self._pumps[i].sendCommand('RAT {} {}'.format(str(100), 'UM'))
                self._pumps[i].sendCommand('VOL {}'.format(str(34)))
                self._pumps[i].sendCommand('DIR INF')

                self._pumps[i].sendCommand('PHN  7')  # reg flow 2
                self._pumps[i].sendCommand('FUN RAT')
                self._pumps[i].sendCommand('RAT {} {}'.format(str(3), 'UM'))
                self._pumps[i].sendCommand('VOL {}'.format(str(59)))
                self._pumps[i].sendCommand('DIR INF')

                self._pumps[i].sendCommand('PHN  8')  # pulse flow 3
                self._pumps[i].sendCommand('FUN RAT')
                self._pumps[i].sendCommand('RAT {} {}'.format(str(100), 'UM'))
                self._pumps[i].sendCommand('VOL {}'.format(str(34)))
                self._pumps[i].sendCommand('DIR INF')

                self._pumps[i].sendCommand('PHN  9')  # reg flow 3
                self._pumps[i].sendCommand('FUN RAT')
                self._pumps[i].sendCommand('RAT {} {}'.format(str(3), 'UM'))
                self._pumps[i].sendCommand('VOL {}'.format(str(int(round(3*(19.5-pasT),0)))))
                self._pumps[i].sendCommand('DIR INF')

                self._pumps[i].sendCommand('PHN  10')
                self._pumps[i].sendCommand('FUN LOP 15')
                # print('phase 4')

                self._pumps[i].sendCommand('PHN  11')
                self._pumps[i].sendCommand('FUN STP')

                self._pumps[i].run()

            elif self._prog[i] == 'wash': # not complete, hardcoded, not using json
                #loopVol = pulseRate*pulseTime + flowRate*flowTime
                #loops = 10 # self.vole[i] / loopVol

                pasT = 5 #min
                self._pumps[i].sendCommand('PHN  1')
                self._pumps[i].sendCommand('FUN RAT')
                self._pumps[i].sendCommand('RAT {} {}'.format(str(50),'UM'))
                self._pumps[i].sendCommand('VOL {}'.format(str(int(round(50*pasT,0)))))
                self._pumps[i].sendCommand('DIR INF')


                self._pumps[i].sendCommand('PHN  2')
                self._pumps[i].sendCommand('FUN LPS')

                self._pumps[i].sendCommand('PHN  3')
                self._pumps[i].sendCommand('FUN PAS 60') #change loop time

                self._pumps[i].sendCommand('PHN  4')
                self._pumps[i].sendCommand('FUN LOP 55') # loop the 60s pause 55 times

                self._pumps[i].sendCommand('PHN  5')
                self._pumps[i].sendCommand('FUN LOP 15') #15 hour loop

                self._pumps[i].sendCommand('PHN  6')
                self._pumps[i].sendCommand('FUN STP')
                self._pumps[i].run()
            else:
                if self._pumps[i].getStatus() != 'halted':
                    self._pumps[i].stop()

        else:
            if self._pumps[i].getStatus() != 'halted':
                self._pumps[i].stop()

    def run_pump_manual(self, i): ## STILL NEED TO ADD: if there is not rate entered do nothing
        print('trying pump manual {} '.format(i))
        try:
            # test = int(self.vol[i].text()) + 1
            # print('vol is {}'.format(self.vol[i].text()))
            # print(isinstance(int(self.vol[i].text())   , int))
            if int(self.vol[i].text()) == 0 or  self.vol[i].text() == '':
                raise ValueError
            if self.run_man_btns[i].isChecked():
                print('run man pump {}'.format(i))
                if self.run_btns[i].isChecked():
                    self.run_btns[i].setChecked(False)
                if self._pumps[i].getStatus() != 'halted':
                    self._pumps[i].stop()
                print('rate is {}'.format(str(self.rates[i].text())))
                print('vol to dispense {}'.format(str(self.vol[i].text())))

                self._pumps[i].setRate(str(self.rates[i].text()))
                self._pumps[i].setVolume(str(self.vol[i].text()))
                self._pumps[i].run() # probably dont need to make into str
                print(type(self.vol[i].text()))
            else:
                if self._pumps[i].getStatus() != 'halted':
                    self._pumps[i].stop()
        except ValueError:
             self.error_state = 'volume  for pump {} must be a non zero int'.format(i)
             print('volume  for pump {} must be a non zero int'.format(i))
             self.run_man_btns[i].setChecked(False)
             self.errorbar.setText('Error: ' + self.error_state)

    def stop_pump(self, i):
        print('Trying to stop pump {} '.format(i))
        self.run_btns[i].setChecked(False)
        self.run_man_btns[i].setChecked(False)
        if self._pumps[i].getStatus() != 'halted':
            print('Stopping pump {} '.format(i))
            self._pumps[i].stop()
            #self.run_btns[i].setChecked(False)

    def check_rates_loop(self):
        for i,p in enumerate(self._pumps):
            #voldisp = p.get
            stat = p.getStatus()
            if stat != 'infusing' and stat != 'withdrawing':
                self.currflow[i].setText(str(0))
            else:
                self.currflow[i].setText(p.getRate())
        self.t = threading.Timer(self._update_status_time,self.check_rates_loop)
        self.t.start()

def main_ui():
    try:
        if (len(sys.argv)>1):
            fp = open(sys.argv[1])
        else:
            fp = open('mypumps.json')
        pump_config = json.load(fp)
        fp.close()
    except IOError:
        print ('config file not found')
        sys.exit(0)

    programs = {x['name']:x for x in pump_config['programs']}
    ser = serial.Serial(baudrate=19200,timeout=0.1,port='COM1')
    print(ser.is_open)

    pumps = []
    for c in pump_config['pumps']:
        pumps.append(Pump(ser,c))

    app = QtWidgets.QApplication(sys.argv)
    ex = PumpControl(ser,pumps,programs)
    ret = app.exec_()
    ex.t.cancel()
    sys.exit(ret)


def main_test(sleeptime=None):
    try:
        if (len(sys.argv)>1):
            fp = open(sys.argv[1])
        else:
            fp = open('mypumps.json')
        pump_config = json.load(fp)
        fp.close()
    except IOError:
        print ('config file not found')
        sys.exit(0)

    ser = serial.Serial(baudrate=19200,timeout=0.1,port='COM1')
    print(ser.is_open)

    pumps = []
    for c in pump_config['pumps']:
        pumps.append(Pump(ser,c))

    for p in pumps:
        p.setDirection('infuse')
        #p.run()


        p.sendCommand('PHN  1')
        p.sendCommand('FUN RAT')
        p.sendCommand('RAT {} {}'.format(str(10), 'UM'))
        p.sendCommand('VOL {}'.format(str(5)))
        p.sendCommand('DIR INF')
        # ('phase 2')

        p.sendCommand('PHN  2')
        p.sendCommand('FUN RAT')
        p.sendCommand('RAT {} {}'.format(str(150), 'UM'))
        p.sendCommand('VOL {}'.format(str(50)))
        p.sendCommand('DIR INF')
        # print('phase 3')

        p.sendCommand('PHN  3')
        p.sendCommand('FUN LOP 2')
        # print('phase 4')

        p.sendCommand('PHN  4')
        p.sendCommand('FUN STP')
        p.run()
    allrates = [[]]*3

    for l in range(30):# get rate
        for i,p in enumerate(pumps):
            allrates[i].append(p.getRate())   # (str(p.getAddress()) +' '+ str(p.getRate()))
        time.sleep(2)
    print(allrates)


        #print(p.getDirection())

    # if sleeptime is not None:
    #     print('running...')
    #     time.sleep(sleeptime)
    #     print('timer done')
    #     for p in pumps:
    #         print(p.getStatus())
    #         if p.getStatus() != 'halted':
    #             p.stop()
    #             print('I had to stop pump {} at exit.'.format(p.getAddress()))
    #     print('pump at ADR {} infused {}'.format(p.getAddress(),p.getInfused()))

    # ser.close()
    # print(ser.is_open)
    for p in pumps:
        p.stop()

    ser.close()

    return ser, pumps


if __name__ == '__main__':
    #main_test(10)
    main_ui()