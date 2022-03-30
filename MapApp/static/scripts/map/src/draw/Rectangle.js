class Rectangle extends DrawManager
{
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'Rectangle', name, parameters, options, layerOptions);
    }

    codeDraw(values, options={}) {
       throw new Error("Not implemented");
    }
}
