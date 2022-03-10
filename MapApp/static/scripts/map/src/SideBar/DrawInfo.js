class DrawInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-drawInfo-content';
        this.layerList = new SmartList();
        M.addMapCallback('pm:create', this.createLayer.bind(this));
    }

    createLayer(myargs, layer) {

        let options = layer.layer.pm.options.DrawManager;

        if (options.missionPlanner) {
            let drawManager = options.instance;
            let drawInfo = drawManager.getHtmlDrawInfo(this.htmlId, layer);
            HTMLUtils.addToExistingElement(`${this.htmlId}`, [drawInfo]);
            console.log("drawInfo");
            console.log(drawInfo);
        }
    }
}