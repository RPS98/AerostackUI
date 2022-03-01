class DrawInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-drawInfo-content';
        M.addPmCreateCallback(this.updateLayers.bind(this));
    }

    updateLayers(myargs, layer) {
        console.log("updateLayers");
        console.log(layer);
    }
}