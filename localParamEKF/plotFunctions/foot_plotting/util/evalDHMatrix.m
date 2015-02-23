function G = evalDHMatrix(a, d, alph, thet)
    G = [            cos(thet), -sin(thet)*cos(alph),  sin(thet)*sin(alph),          cos(thet)*a
                sin(thet),  cos(thet)*cos(alph), -cos(thet)*sin(alph),          sin(thet)*a
                        0,            sin(alph),            cos(alph),                    d
                        0,                    0,                    0,                    1];
end