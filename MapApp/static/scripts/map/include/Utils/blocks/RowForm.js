class RowForm extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'list': args[0],
            }
        );
    }   

    static addHTML(content) {
        let div = document.createElement('div');
        div.setAttribute('class', 'row');

        let form = document.createElement('form');

        let contentCol = 12 / content.list.length;

        for (let i = 0; i < content.list.length; i++) {

            if (content.list[i].type != 'button') {

                console.log("RowForm for i = " + i);
                console.log(content.list[i].attributes['class']);
                

                content.list[i].attributes['class'] += ' col-' + contentCol;
            }
            
            HTMLUtils.addHTML(form, content.list[i]);
        }
        div.appendChild(form);
        return div;
    }
}