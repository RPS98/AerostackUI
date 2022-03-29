class Circle extends DrawManager
{
    constructor(status, name, options = undefined, layerOptions = undefined) {
        super(status, 'Circle', name, options, layerOptions);
    }
}

class CircularArea extends Circle
{
    constructor(status, options = undefined, layerOptions = undefined) {
        super(status, 'CircularArea', options, layerOptions);
    }
}
