class DrawInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-drawInfo-content';
        this.layerList = new SmartList();
        M.addMapCallback('pm:create', this.createLayer.bind(this));

        M.DRAW_LAYERS.addInfoAddCallback(this.updateLayerCallback.bind(this));
    }

    updateLayerCallback(myargs, args) {
        console.log("DrawInfo: updateLayerCallback")
        console.log(args)
    }

    createLayer(myargs, layer) {

        let options = layer.layer.pm.options.DrawManager;

        if (options.missionPlanner) {
            let drawManager = options.instance;
            let drawInfo = drawManager.getHtmlDrawInfo(this.htmlId, layer);
            HTMLUtils.addToExistingElement(`${this.htmlId}`, [drawInfo]);
        }
    }
}