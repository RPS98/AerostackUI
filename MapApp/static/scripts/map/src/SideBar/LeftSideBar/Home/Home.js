class Home
{
    constructor() {
        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        let homeHtmlList = [];

        // Go to
        let latLonAttributes = {
            'class': 'form-control',
            'required': 'required',
        };
        let latLonBtnAttributes = {
            'class': 'btn btn-primary',
        }
        let goToLat  = HTMLDicts.addInput('text',   'latitude',  'sideBar-left-home-content-goToLat', latLonAttributes);
        let goToLong = HTMLDicts.addInput('input',  'longitude', 'sideBar-left-home-content-goToLon', latLonAttributes);
        let goToBtn  = HTMLDicts.addButton('Go to', 'sideBar-left-home-content-goToBtn', latLonBtnAttributes);
        let goToRow  = HTMLDicts.addRow([goToLat, goToLong, goToBtn]);
        let gotoCollapse = HTMLDicts.addCollapse('Go to', [goToRow], 'sideBar-left-home-content-goToCollapse');
        homeHtmlList.push(gotoCollapse);

        let homeHtml = HTMLDicts.addExistingElement('sideBar-left-home-content', homeHtmlList);

        HTMLBlocks.addDict(homeHtml);
    }

    addCallbacks() {
        Utils.addFormCallback(
            'sideBar-left-home-content-goToBtn', 
            ['sideBar-left-home-content-goToLat', 'sideBar-left-home-content-goToLon'], 
            ['map_center_lat', 'map_center_long'], 
            Home.goToCallback,
            19 // zoom
        );
    }

    static goToCallback(zoom, input) {
        console.log('goToCallback');
        console.log(map)
        map.flyTo([input['map_center_lat'], input['map_center_long']], zoom);
    }
}