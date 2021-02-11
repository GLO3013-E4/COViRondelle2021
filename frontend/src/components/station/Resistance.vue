<template>
  <v-card>
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">
        Station de résistance
      </h5>
    </v-card-title>
    <v-container>
      <v-row align="center">
        <v-col sm="5">
          <div class="d-flex justify-center">{{ this.resistanceValue }} Ω</div>
        </v-col>
        <v-col sm="7">
          <v-avatar
            size="30"
            v-for="(puckColor, i) in pucksColors"
            :key="i"
            :color="puckColor"
            class="cyan--text"
          >
            {{ i + 1 }}
          </v-avatar>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue, Prop } from "vue-property-decorator";

@Component({
  components: {}
})
export default class Resistance extends Vue {
  @Prop() readonly resistanceValue!: number;
  @Prop() readonly pucksColors!: Array<string>;
}
</script>
