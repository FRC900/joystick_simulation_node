from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import sys
from enum import Enum



class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3

class Joystick(QWidget):
    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(150, 150)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 50

    def paintEvent(self, event):
        painter = QPainter(self)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())



    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        return QPointF(self.width()/2, self.height()/2)


    def _boundJoystick(self, point):
  
        limitLine = QLineF(self._center(), point)
        if (limitLine.length() > self.__maxDistance):
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()



    def joystickDirection(self):
        if not self.grabCenter:
            return [0,0]
        normVector = QLineF(self._center(), self.movingOffset)
        result = []
        result.append(((normVector.x2() - normVector.x1()) / 50))
        #print(normVector.x2())
        #print(normVector.x1())
        if normVector.x2() < 1 :
            result[0] = 0
        result.append(-1 * (((normVector.y2() - normVector.y1()) / 50)))
        #print(normVector.y2())
        #print(normVector.y1())
        if normVector.y2() < 1 :
            result[1] = 0
        return result
 

    
    def move_joystick(self, x, y):
        x_y = QPointF(x,y)
        self.movingOffset = self._boundJoystick(x_y)
        self.grabCenter = True
        self.update()

    
    def set_zero(self):
        self.grabCenter = False
        self.movingOffset = QPointF(0,0)
        self.update()
    
    
    

    

    #mouse press event
    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    #figure out what mouse release event is 
    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.update()

    #figure out what mouse press event does relative to toehr commands
    def mouseMoveEvent(self, event):
        self.movingOffset = self._boundJoystick(event.pos())
        

        self.grabCenter = True
        self.update()


    
    



if __name__ == '__main__':
    # Create main application window
    app = QApplication([])
    app.setStyle(QStyleFactory.create("Cleanlooks"))
    mw = QMainWindow()
    mw.setWindowTitle('Joystick example')

    # Create and set widget layout
    # Main widget container
    cw = QWidget()
    ml = QGridLayout()
    cw.setLayout(ml)
    mw.setCentralWidget(cw)

    # Create joystick
    joystick = Joystick()

    # ml.addLayout(joystick.get_joystick_layout(),0,0)
    ml.addWidget(joystick,0,0)

    mw.show()

    ## Start Qt event loop unless running in interactive mode or using pyside.
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()