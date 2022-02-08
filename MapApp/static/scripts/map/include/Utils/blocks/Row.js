class Row extends HTMLUtilsPrototype 
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
        let len = content.list.length;
        let div = document.createElement('div');
        div.setAttribute('class', 'row my-1 mx-1');

        let col = 12 / len;

        for (let i = 0; i < len; i++) {
            let div_col = document.createElement('div');
            div_col.setAttribute('class', `col-md-${col}`);
            HTMLUtils.addHTML(div_col, content.list[i]);
            div.appendChild(div_col);
        }

        return div;
    }
}