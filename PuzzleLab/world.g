world{ X:[0 0 .85]}

base(world){ Q:[0.0 0.0 0.1]}
ego(base){
    shape:ssBox, size:[.03 .03 .03 .001], color:[1 0 0], limits:[-1 1 -1 1 -3 3]
    joint:transXYPhi, contact: 1
}
