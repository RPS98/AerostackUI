class Home
{
    constructor()
    {

        let goToLat  = HTMLUtils.addInputDict('sideBar-left-home-content-goToLat', 'text', 'latitude');
        let goToLong = HTMLUtils.addInputDict('sideBar-left-home-content-goToLon', 'input', 'longitude');
        let goToBtn  = HTMLUtils.addInputDict('sideBar-left-home-content-goToBtn', 'button', 'Go to');

        let goToRow = HTMLUtils.addRow([goToLat, goToLong, goToBtn]);

        let gotoCollapse = HTMLUtils.addCollapse('auto', 'Go to', [goToRow]);

        let homeHtml = {
            'container_id': 'sideBar-left-home-content',
            'content': [gotoCollapse]
        };

        HTMLUtils.addContent(homeHtml);

    }
}