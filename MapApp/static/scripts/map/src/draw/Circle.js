class Circle extends DrawManager
{
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'Circle', name, parameters, options, layerOptions);
    }
}

class CircularArea extends Circle
{
    constructor(status, options = undefined, layerOptions = undefined, parameters = undefined) {
        super(status, 'CircularArea', parameters, options, layerOptions);
    }
}
