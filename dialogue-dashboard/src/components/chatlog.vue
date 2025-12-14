<template>
    <v-row>
        <v-col cols="12" sm="9">
            <v-card>
                <v-card-title class="black red--text">
                    {{ showMessage }}
                </v-card-title>
                <v-virtual-scroll
                    :items="chatLog"
                    :item-height="60"
                    height="600"
                >
                    <template  v-slot="{ item }">
                        <v-divider :insert="true" dark></v-divider>
                        <v-list-item :class="bgColor(item.turn)">
                            <v-list-item-icon>
                                <v-icon>{{ userIcon(item.turn) }}</v-icon>
                            </v-list-item-icon>
                            <v-list-item-content>
                                <v-list-item-title :class="contentColor(item.turn)">{{ item.log }}</v-list-item-title>
                            </v-list-item-content>
                        </v-list-item>
                    </template>
                </v-virtual-scroll>
            </v-card>
        </v-col>
        <v-col cols="12" sm="3">

        </v-col>
        <v-col cols="12" sm="10" offset="1">
            <v-textarea
                v-if="debug"
                outlined
                label="Development Log"
                filled
                no-resize
                :value="devLog"
            >
            </v-textarea>
        </v-col>
    </v-row>
</template>

<script lang="ts">
import { Vue, Component, Watch } from 'vue-property-decorator';
import $ from 'jquery';

@Component({})
export default class ChatLog extends Vue {
    private turn = 0;
    private speaker = '';
    private bot = '';
    private chatLog: { turn: number, log: string }[] = [{ turn: 0, log: ''}];
    private debug = true;
    private eventId = 0;

    private get showMessage(): string {
        return this.turn == 0 ? `YOU:　${this.speaker}` : `BOT: ${this.bot}`; 
    }

    private userIcon( turn: number ): string {
        return turn == 0 ? "mdi-account" : "mdi-robot";
    }

    private contentColor( turn: number ): string {
        return turn == 0 ? "title" : "title";
    }

    private bgColor( turn: number ): string {
        return turn == 0 ? "grey" : "white";
    }

    private get devLog(): string {
        return `chatLog: ${JSON.stringify(this.chatLog)}`;
    }

    async created() {
        this.eventId = setInterval(this.update, 100);
    }

    private beforeDestroy() {
        clearInterval(this.eventId);
    }

    private async update() {
        const result = $.ajax({
            url: '/getLog',
            type: 'GET',
            dataType: 'json'
        }).done((res:any) =>{
            const num = Number(res.data.turn);
            this.speaker = res.data.speaker;
            this.bot = res.data.bot;
            if( this.turn == num ){
                this.chatLog.splice( this.chatLog.length - 1, 1,  num === 0 ? 
                { turn: num, log: this.speaker } : { turn: num, log: this.bot } );
            } else {
                this.chatLog.push( num === 0 ? 
                { turn: num, log: this.speaker } : { turn: num, log: this.bot } );
            }
            this.turn = num;
            console.log(this.chatLog);
        }).fail(() => {
            console.log("err");
        });
    }
}

</script>