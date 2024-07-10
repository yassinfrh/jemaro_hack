import cv2
import numpy as np
import sys
import math
from matplotlib import pyplot as plt

def segmentColor(img, color, threshold = 2):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, np.array([color-threshold, 70, 70]), np.array([color+threshold, 255, 255]))        
    
    mask = cv2.dilate(mask, np.ones((3,3)))
    mask = cv2.dilate(mask, np.ones((3,3)))
    mask = cv2.dilate(mask, np.ones((3,3)))

    mask = cv2.erode(mask, np.ones((3,3)))
    mask = cv2.erode(mask, np.ones((3,3)))
    mask = cv2.erode(mask, np.ones((3,3)))
    
    mask = cv2.erode(mask, np.ones((3,3)))
    mask = cv2.erode(mask, np.ones((3,3)))
    mask = cv2.erode(mask, np.ones((3,3)))
    
    mask = cv2.dilate(mask, np.ones((3,3)))
    mask = cv2.dilate(mask, np.ones((3,3)))
    mask = cv2.dilate(mask, np.ones((3,3)))

    return mask.astype(np.uint8)

def contourProps(contour):
    mu = cv2.moments(contour)
    cc = (int(mu['m10'] / (mu['m00'] + 1e-5)), int(mu['m01'] / (mu['m00'] + 1e-5)))
    ca = cv2.contourArea(contour)
    cl = cv2.arcLength(contour, True)

    ellipse = cv2.fitEllipse(contour)

    return {'ca': ca, 'cc': cc, 'el': ellipse}

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def countCorners(contour, props):
    from scipy.signal import find_peaks

    # get contour points in polar coordinates
    [cx, cy] = props['cc']
    ds, phis = [], []
    for i in range(len(contour)):
        x, y = contour[i][0]
        d, rho = cart2pol(x-cx, y-cy)
        ds.append(d)
        phis.append(rho)

    
    max_id = np.argmax(ds)
    ori = phis[max_id]

    min_id = np.argmin(ds)
    ds = np.roll(ds, -min_id)
    phis = np.roll(phis, -min_id)
    phis = np.unwrap(phis)
    phis = phis - min(phis)

    ds = cv2.blur(ds, (1,15))
    ds = np.reshape(ds, (ds.shape[0],))
    pks, _ = find_peaks(ds, width=10, prominence=5)


    rad = np.mean(ds[pks]) if len(pks) > 0 else np.mean(ds)

    return [len(pks), ori, rad]

def detectShapes(img, colors):
    shapes = []

    for c in colors:
        mask = segmentColor(img, colors[c])
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


        for cnt in contours:
            cp = contourProps(cnt)

            # reject small objects
            if cp['ca'] < 300:
                continue

            # reject non-regular (elongated) shapes
            [rx, ry] = cp['el'][1]
            if min(rx, ry) / max(rx, ry) < 0.8:
                continue

            [corners, ori, rad] = countCorners(cnt, cp)

            shape = {
                'color': c,
                'center': cp['cc'],
                'corners': corners,
                'ori': ori,
                'rad': rad,
                'cnt': cnt
            }

            shapes.append(shape)

    return shapes

def drawShapes(img, shapes, colors):
    out = img.copy()# // 3

    for shp in shapes:
        cc = shp['center']
        
        cv2.drawContours(out, [shp['cnt']], -1, (0, 0, 0), 2)

        x = int(cc[0] + shp['rad'] * math.cos(shp['ori']))
        y = int(cc[1] + shp['rad'] * math.sin(shp['ori']))

        cv2.line(out, cc, [x, y], (255, 255, 255), 1)

        (px, py, pz) = shp['pos']

        cv2.putText(out, f"{round(shp['ori'], 2)} | {shp['color']} | {shp['corners']} | {px:.2f}, {py:.2f}, {pz:.2f}", (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255))

    return out

def localizeShapes(shapes, radius, cameraInfo):
    out = []
    fx = cameraInfo['k'][0]
    fy = cameraInfo['k'][4]
    cx = cameraInfo['k'][2]
    cy = cameraInfo['k'][5]

    for shp in shapes:
        z = 0.3  #fx * radius / shp['rad']
        x = z * (shp['center'][0]-cx) / fx
        y = z * (shp['center'][1]-cy) / fy

        shp['pos'] = (x, y, z)
        out.append(shp)

    return out

if __name__=="__main__":

    fname = sys.argv[1]
    '''colors = {
        "yellow": [49, 128, 144],
        "navy": [64, 32, 0],
        "purple": [53, 32, 66],
        "green": [72, 103, 11],
        "blue": [128, 64, 0],
        "red": [0, 0, 128]
    }'''

    colors = {
        "yellow": 60,
        "navy": 240,
        "purple": 300,
        "green": 120,
        "blue": 210,
        "red": 0
    }

    cameraInfo = {
        'k': [907.4107666015625, 0.0, 648.4189453125,
              0.0, 907.2595825195312, 357.08447265625,
              0.0, 0.0, 1.0]
    }

    img = cv2.imread(fname)

    # out = cv2.Canny(img, 30, 90)
    # out = cv2.dilate(out, np.ones((5,5)))
    # contours, hierarchy = cv2.findContours(out, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    # cv2.imshow("can", 255-out)
    # cv2.waitKey(-1)

    refRadius = 0.05
    shapes = detectShapes(img, colors)
    shapes = localizeShapes(shapes, refRadius, cameraInfo)
    out = drawShapes(img, shapes, colors)
    print(shapes)

    cv2.imwrite("/tmp/shapes.png", out)

