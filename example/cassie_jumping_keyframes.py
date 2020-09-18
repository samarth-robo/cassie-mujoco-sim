import os
import numpy as np

file_path = os.path.dirname(os.path.realpath(__file__))

# =========== evenly spaced key frames ===========
keyframes_txt = [
# crounch down
'-6.04788e-05 6.09942e-09 0.997646 1 -2.41486e-07 -0.000422966 -5.81877e-07 0.00450306 0.000159769 0.498253 0.978385 -0.0163951 0.0179253 -0.20536 -1.20037 -0.00265447 1.42545 -0.00136681 -1.50486 1.48647 -1.61169 -0.00450246 -0.000158607 0.498253 0.978515 0.00385628 -0.0152923 -0.205572 -1.20037 -0.00265299 1.42545 -0.0013666 -1.50486 1.48647 -1.61169',
'-0.00253404 -5.92364e-07 0.90924 0.999674 -3.44132e-05 -0.0255494 -3.06299e-05 0.00532022 0.00522507 0.583396 0.938194 -0.0148536 0.033628 -0.344152 -1.49103 -0.00400188 1.72 -0.00338209 -1.58212 1.56369 -1.69251 -0.00518715 -0.00513394 0.583389 0.938317 0.00283055 -0.0292123 -0.344528 -1.49097 -0.0040025 1.71995 -0.00338413 -1.58211 1.56368 -1.6925',
'-0.00646847 -3.57456e-07 0.734525 0.996835 -9.74397e-05 -0.0794982 -8.37272e-05 0.00821777 0.0102183 0.685732 0.829028 -0.00917545 0.0640039 -0.555456 -1.9727 -0.00505215 2.20368 -0.00495822 -1.67184 1.65341 -1.78153 -0.00784484 -0.0100292 0.685756 0.82911 -0.00144627 -0.0568742 -0.556183 -1.97258 -0.00505348 2.20357 -0.00496074 -1.67183 1.6534 -1.78152',
# stand up
'-0.00253404 -5.92364e-07 0.90924 0.999674 -3.44132e-05 -0.0255494 -3.06299e-05 0.00532022 0.00522507 0.583396 0.938194 -0.0148536 0.033628 -0.344152 -1.49103 -0.00400188 1.72 -0.00338209 -1.58212 1.56369 -1.69251 -0.00518715 -0.00513394 0.583389 0.938317 0.00283055 -0.0292123 -0.344528 -1.49097 -0.0040025 1.71995 -0.00338413 -1.58211 1.56368 -1.6925',
'-6.04788e-05 6.09942e-09 0.997646 1 -2.41486e-07 -0.000422966 -5.81877e-07 0.00450306 0.000159769 0.498253 0.978385 -0.0163951 0.0179253 -0.20536 -1.20037 -0.00265447 1.42545 -0.00136681 -1.50486 1.48647 -1.61169 -0.00450246 -0.000158607 0.498253 0.978515 0.00385628 -0.0152923 -0.205572 -1.20037 -0.00265299 1.42545 -0.0013666 -1.50486 1.48647 -1.61169',
# take off
'-4.36493e-05 1.22361e-08 1.16075 1 -2.18936e-07 -0.000381918 -4.59498e-07 0.00449009 7.25645e-05 0.497796 0.978606 -0.0164018 0.0178207 -0.204315 -1.20034 -7.41517e-05 1.42182 -5.99219e-05 -1.50673 1.48832 -1.61458 -0.00448961 -7.20957e-05 0.497797 0.978735 0.00386015 -0.015201 -0.204526 -1.20034 -7.40389e-05 1.42182 -5.97536e-05 -1.50673 1.48832 -1.61458',
'-2.30774e-05 1.94524e-08 1.21337 1 -1.98572e-07 -0.000371295 -3.51424e-07 0.00448624 3.03723e-05 0.497377 0.978649 -0.016403 0.0177993 -0.20411 -1.19997 7.00904e-07 1.42141 7.73257e-07 -1.50672 1.48832 -1.61456 -0.0044859 -3.04137e-05 0.497377 0.978779 0.00386076 -0.0151823 -0.204321 -1.19997 7.00421e-07 1.42141 7.71031e-07 -1.50672 1.48832 -1.61456',
# fall down
'-5.19806e-06 2.3237e-08 1.16788 1 -1.63704e-07 -0.000332341 -2.46474e-07 0.00448539 1.41259e-05 0.497193 0.978668 -0.0164035 0.01779 -0.204019 -1.19978 3.17891e-07 1.42122 1.71509e-07 -1.50672 1.48832 -1.61456 -0.00448519 -1.43599e-05 0.497194 0.978798 0.00386109 -0.0151741 -0.20423 -1.19978 3.17369e-07 1.42122 1.7164e-07 -1.50672 1.48832 -1.61456',
'1.18368e-05 2.49443e-08 1.02428 1 -1.27309e-07 -0.000278932 -1.40814e-07 0.0044856 7.80135e-06 0.497112 0.978677 -0.0164038 0.0177856 -0.203976 -1.19969 1.52187e-07 1.42113 7.7308e-08 -1.50672 1.48832 -1.61456 -0.0044855 -8.10564e-06 0.497112 0.978807 0.00386124 -0.0151703 -0.204187 -1.19969 1.51893e-07 1.42113 7.7474e-08 -1.50672 1.48832 -1.61456',
'-0.00645811 -1.16589e-06 0.801438 0.997959 -7.17901e-05 -0.0638645 -7.71558e-05 0.00746802 0.0128497 0.696146 0.862181 -0.0109993 0.0556477 -0.503415 -1.8445 -0.00868471 2.07794 -0.00662283 -1.66654 1.64807 -1.77309 -0.00718889 -0.0126209 0.696163 0.862274 -4.76171e-05 -0.0491874 -0.504048 -1.84441 -0.00868589 2.07786 -0.00662486 -1.66652 1.64805 -1.77307',
# stand up
'-0.00253404 -5.92364e-07 0.90924 0.999674 -3.44132e-05 -0.0255494 -3.06299e-05 0.00532022 0.00522507 0.583396 0.938194 -0.0148536 0.033628 -0.344152 -1.49103 -0.00400188 1.72 -0.00338209 -1.58212 1.56369 -1.69251 -0.00518715 -0.00513394 0.583389 0.938317 0.00283055 -0.0292123 -0.344528 -1.49097 -0.0040025 1.71995 -0.00338413 -1.58211 1.56368 -1.6925',
'-6.04788e-05 6.09942e-09 0.997646 1 -2.41486e-07 -0.000422966 -5.81877e-07 0.00450306 0.000159769 0.498253 0.978385 -0.0163951 0.0179253 -0.20536 -1.20037 -0.00265447 1.42545 -0.00136681 -1.50486 1.48647 -1.61169 -0.00450246 -0.000158607 0.498253 0.978515 0.00385628 -0.0152923 -0.205572 -1.20037 -0.00265299 1.42545 -0.0013666 -1.50486 1.48647 -1.61169',
]

keyframes = np.empty((0,35), dtype=float)
for i in keyframes_txt:
    keyframes = np.append(keyframes, np.array(i.split(), dtype=float).reshape(1,-1), axis=0)

keyframes.tofile(file_path+"/"+"keyframes_even.bin")

keyframes = np.fromfile(file_path+"/"+"keyframes_even.bin", dtype=float)
keyframes = keyframes.reshape(-1,35)


# =========== unevenly spaced key frames ===========
keyframes_txt = [
# crounch down
'-6.04788e-05 6.09942e-09 0.997646 1 -2.41486e-07 -0.000422966 -5.81877e-07 0.00450306 0.000159769 0.498253 0.978385 -0.0163951 0.0179253 -0.20536 -1.20037 -0.00265447 1.42545 -0.00136681 -1.50486 1.48647 -1.61169 -0.00450246 -0.000158607 0.498253 0.978515 0.00385628 -0.0152923 -0.205572 -1.20037 -0.00265299 1.42545 -0.0013666 -1.50486 1.48647 -1.61169',
'-0.00646847 -3.57456e-07 0.734525 0.996835 -9.74397e-05 -0.0794982 -8.37272e-05 0.00821777 0.0102183 0.685732 0.829028 -0.00917545 0.0640039 -0.555456 -1.9727 -0.00505215 2.20368 -0.00495822 -1.67184 1.65341 -1.78153 -0.00784484 -0.0100292 0.685756 0.82911 -0.00144627 -0.0568742 -0.556183 -1.97258 -0.00505348 2.20357 -0.00496074 -1.67183 1.6534 -1.78152',
# stand up
'-0.00253404 -5.92364e-07 0.90924 0.999674 -3.44132e-05 -0.0255494 -3.06299e-05 0.00532022 0.00522507 0.583396 0.938194 -0.0148536 0.033628 -0.344152 -1.49103 -0.00400188 1.72 -0.00338209 -1.58212 1.56369 -1.69251 -0.00518715 -0.00513394 0.583389 0.938317 0.00283055 -0.0292123 -0.344528 -1.49097 -0.0040025 1.71995 -0.00338413 -1.58211 1.56368 -1.6925',
# take off
'-2.30774e-05 1.94524e-08 1.21337 1 -1.98572e-07 -0.000371295 -3.51424e-07 0.00448624 3.03723e-05 0.497377 0.978649 -0.016403 0.0177993 -0.20411 -1.19997 7.00904e-07 1.42141 7.73257e-07 -1.50672 1.48832 -1.61456 -0.0044859 -3.04137e-05 0.497377 0.978779 0.00386076 -0.0151823 -0.204321 -1.19997 7.00421e-07 1.42141 7.71031e-07 -1.50672 1.48832 -1.61456',
# fall down
'-5.19806e-06 2.3237e-08 1.16788 1 -1.63704e-07 -0.000332341 -2.46474e-07 0.00448539 1.41259e-05 0.497193 0.978668 -0.0164035 0.01779 -0.204019 -1.19978 3.17891e-07 1.42122 1.71509e-07 -1.50672 1.48832 -1.61456 -0.00448519 -1.43599e-05 0.497194 0.978798 0.00386109 -0.0151741 -0.20423 -1.19978 3.17369e-07 1.42122 1.7164e-07 -1.50672 1.48832 -1.61456',
'-0.00645811 -1.16589e-06 0.801438 0.997959 -7.17901e-05 -0.0638645 -7.71558e-05 0.00746802 0.0128497 0.696146 0.862181 -0.0109993 0.0556477 -0.503415 -1.8445 -0.00868471 2.07794 -0.00662283 -1.66654 1.64807 -1.77309 -0.00718889 -0.0126209 0.696163 0.862274 -4.76171e-05 -0.0491874 -0.504048 -1.84441 -0.00868589 2.07786 -0.00662486 -1.66652 1.64805 -1.77307',
# stand up
'-0.00253404 -5.92364e-07 0.90924 0.999674 -3.44132e-05 -0.0255494 -3.06299e-05 0.00532022 0.00522507 0.583396 0.938194 -0.0148536 0.033628 -0.344152 -1.49103 -0.00400188 1.72 -0.00338209 -1.58212 1.56369 -1.69251 -0.00518715 -0.00513394 0.583389 0.938317 0.00283055 -0.0292123 -0.344528 -1.49097 -0.0040025 1.71995 -0.00338413 -1.58211 1.56368 -1.6925',
]

keyframes = np.empty((0,35), dtype=float)
for i in keyframes_txt:
    keyframes = np.append(keyframes, np.array(i.split(), dtype=float).reshape(1,-1), axis=0)

keyframes.tofile(file_path+"/"+"keyframes_uneven.bin")

keyframes = np.fromfile(file_path+"/"+"keyframes_uneven.bin", dtype=float)
keyframes = keyframes.reshape(-1,35)