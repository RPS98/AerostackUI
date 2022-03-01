class DrawInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-drawInfo-content';
        M.addPmCreateCallback(this.createLayer.bind(this));
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